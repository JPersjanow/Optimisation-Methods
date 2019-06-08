
import os
import sys
try:
    from time import clock
except ImportError:
    from time import perf_counter as clock
from uuid import uuid4
try:
    import configparser
except ImportError:
    import ConfigParser as configparser
try:
    Parser = configparser.ConfigParser
except AttributeError:
    Parser = configparser.SafeConfigParser
from sparse import *
import collections
import warnings
from tempfile import mktemp
from constants import *

import logging
log = logging.getLogger(__name__)

if os.name == "posix" and sys.version_info[0] < 3:
    try:
        import subprocess32 as subprocess
    except ImportError:
        log.debug("Thread-safe subprocess32 module not found! "
                  "Using unsafe built-in subprocess module instead.")
        import subprocess
else:
    import subprocess

class JSolverError(JError):
    """
    Solver-related exceptions
    """
    pass



# See later for LpSolverDefault definition
class LpSolver:
    """A generic LP Solver"""

    def __init__(self, mip = True, msg = True, options = [], *args, **kwargs):
        self.mip = mip
        self.msg = msg
        self.options = options

    def available(self):
        """True if the solver is available"""
        raise NotImplementedError

    def actualSolve(self, lp):
        """Solve a well formulated lp problem"""
        raise NotImplementedError

    def actualResolve(self,lp, **kwargs):
        """
        uses existing problem information and solves the problem
        If it is not implelemented in the solver
        just solve again
        """
        self.actualSolve(lp, **kwargs)

    def copy(self):
        """Make a copy of self"""

        aCopy = self.__class__()
        aCopy.mip = self.mip
        aCopy.msg = self.msg
        aCopy.options = self.options
        return aCopy

    def solve(self, lp):
        """Solve the problem lp"""
        # Always go through the solve method of LpProblem
        return lp.solve(self)

    #TODO: Not sure if this code should be here or in a child class
    def getCplexStyleArrays(self,lp,
                            senseDict={LpConstraintEQ:"E", LpConstraintLE:"L", LpConstraintGE:"G"},
                            LpVarCategories = {LpContinuous: "C",LpInteger: "I"},
                            LpObjSenses = {LpMaximize : -1,
                                           LpMinimize : 1},
                            infBound =  1e20
                            ):
        """returns the arrays suitable to pass to a cdll Cplex
        or other solvers that are similar

        Copyright (c) Stuart Mitchell 2007
        """
        rangeCount = 0
        variables=list(lp.variables())
        numVars = len(variables)
        #associate each variable with a ordinal
        self.v2n=dict(((variables[i],i) for i in range(numVars)))
        self.vname2n=dict(((variables[i].name,i) for i in range(numVars)))
        self.n2v=dict((i,variables[i]) for i in range(numVars))
        #objective values
        objSense = LpObjSenses[lp.sense]
        NumVarDoubleArray = ctypes.c_double * numVars
        objectCoeffs=NumVarDoubleArray()
        #print "Get objective Values"
        for v,val in lp.objective.items():
            objectCoeffs[self.v2n[v]]=val
        #values for variables
        objectConst = ctypes.c_double(0.0)
        NumVarStrArray = ctypes.c_char_p * numVars
        colNames = NumVarStrArray()
        lowerBounds = NumVarDoubleArray()
        upperBounds = NumVarDoubleArray()
        initValues = NumVarDoubleArray()
        for v in lp.variables():
            colNames[self.v2n[v]] = str(v.name)
            initValues[self.v2n[v]] = 0.0
            if v.lowBound != None:
                lowerBounds[self.v2n[v]] = v.lowBound
            else:
                lowerBounds[self.v2n[v]] = -infBound
            if v.upBound != None:
                upperBounds[self.v2n[v]] = v.upBound
            else:
                upperBounds[self.v2n[v]] = infBound
        #values for constraints
        numRows =len(lp.constraints)
        NumRowDoubleArray = ctypes.c_double * numRows
        NumRowStrArray = ctypes.c_char_p * numRows
        NumRowCharArray = ctypes.c_char * numRows
        rhsValues = NumRowDoubleArray()
        rangeValues = NumRowDoubleArray()
        rowNames = NumRowStrArray()
        rowType = NumRowCharArray()
        self.c2n = {}
        self.n2c = {}
        i = 0
        for c in lp.constraints:
            rhsValues[i] = -lp.constraints[c].constant
            #for ranged constraints a<= constraint >=b
            rangeValues[i] = 0.0
            rowNames[i] = str(c)
            rowType[i] = senseDict[lp.constraints[c].sense]
            self.c2n[c] = i
            self.n2c[i] = c
            i = i+1
        #return the coefficient matrix as a series of vectors
        coeffs = lp.coefficients()
        sparseMatrix = sparse.Matrix(list(range(numRows)), list(range(numVars)))
        for var,row,coeff in coeffs:
            sparseMatrix.add(self.c2n[row], self.vname2n[var], coeff)
        (numels, mystartsBase, mylenBase, myindBase,
         myelemBase) = sparseMatrix.col_based_arrays()
        elemBase = ctypesArrayFill(myelemBase, ctypes.c_double)
        indBase = ctypesArrayFill(myindBase, ctypes.c_int)
        startsBase = ctypesArrayFill(mystartsBase, ctypes.c_int)
        lenBase = ctypesArrayFill(mylenBase, ctypes.c_int)
        #MIP Variables
        NumVarCharArray = ctypes.c_char * numVars
        columnType = NumVarCharArray()
        if lp.isMIP():
            for v in lp.variables():
                columnType[self.v2n[v]] = LpVarCategories[v.cat]
        self.addedVars = numVars
        self.addedRows = numRows
        return  (numVars, numRows, numels, rangeCount,
                 objSense, objectCoeffs, objectConst,
                 rhsValues, rangeValues, rowType, startsBase, lenBase, indBase,
                 elemBase, lowerBounds, upperBounds, initValues, colNames,
                 rowNames, columnType, self.n2v, self.n2c)


class LpSolver_CMD(LpSolver):
    """A generic command line LP Solver"""
    def __init__(self, path=None, keepFiles=0, mip=1, msg=1, options=[]):
        LpSolver.__init__(self, mip, msg, options)
        if path is None:
            self.path = self.defaultPath()
        else:
            self.path = path
        self.keepFiles = keepFiles
        self.setTmpDir()

    def copy(self):
        """Make a copy of self"""

        aCopy = LpSolver.copy(self)
        aCopy.path = self.path
        aCopy.keepFiles = self.keepFiles
        aCopy.tmpDir = self.tmpDir
        return aCopy

    def setTmpDir(self):
        """Set the tmpDir attribute to a reasonnable location for a temporary
        directory"""
        if os.name != 'nt':
            # On unix use /tmp by default
            self.tmpDir = os.environ.get("TMPDIR", "/tmp")
            self.tmpDir = os.environ.get("TMP", self.tmpDir)
        else:
            # On Windows use the current directory
            self.tmpDir = os.environ.get("TMPDIR", "")
            self.tmpDir = os.environ.get("TMP", self.tmpDir)
            self.tmpDir = os.environ.get("TEMP", self.tmpDir)
        if not os.path.isdir(self.tmpDir):
            self.tmpDir = ""
        elif not os.access(self.tmpDir, os.F_OK + os.W_OK):
            self.tmpDir = ""

    def defaultPath(self):
        raise NotImplementedError

    def executableExtension(name):
        if os.name != 'nt':
            return name
        else:
            return name+".exe"
    executableExtension = staticmethod(executableExtension)

    def executable(command):
        """Checks that the solver command is executable,
        And returns the actual path to it."""

        if os.path.isabs(command):
            if os.path.exists(command) and os.access(command, os.X_OK):
                return command
        for path in os.environ.get("PATH", []).split(os.pathsep):
            new_path = os.path.join(path, command)
            if os.path.exists(new_path) and os.access(new_path, os.X_OK):
                return os.path.join(path, command)
        return False
    executable = staticmethod(executable)

class GLPK_CMD(LpSolver_CMD):
    """The GLPK LP solver"""
    def defaultPath(self):
        glpk_path = 'glpsol'
        return self.executableExtension(glpk_path)

    def available(self):
        """True if the solver is available"""
        return self.executable(self.path)

    def actualSolve(self, lp):
        """Solve a well formulated lp problem"""
        if not self.executable(self.path):
            raise JSolverError("Error: cannot execute "+self.path)
        if not self.keepFiles:
            uuid = uuid4().hex
            tmpLp = os.path.join(self.tmpDir, "%s-pulp.lp" % uuid)
            tmpSol = os.path.join(self.tmpDir, "%s-pulp.sol" % uuid)
        else:
            tmpLp = lp.name+"-pulp.lp"
            tmpSol = lp.name+"-pulp.sol"
        lp.writeLP(tmpLp, writeSOS = 0)
        proc = ["glpsol", "--cpxlp", tmpLp, "-o", tmpSol]
        if not self.mip: proc.append('--nomip')
        proc.extend(self.options)

        self.solution_time = clock()
        if not self.msg:
            proc[0] = self.path
            pipe = open(os.devnull, 'w')
            if operating_system == 'win':
                # Prevent flashing windows if used from a GUI application
                startupinfo = subprocess.STARTUPINFO()
                startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                rc = subprocess.call(proc, stdout = pipe, stderr = pipe,
                                     startupinfo = startupinfo)
            else:
                rc = subprocess.call(proc, stdout = pipe, stderr = pipe)
            if rc:
                raise JSolverError("Error: Error while trying to execute "+self.path)
            pipe.close()
        else:
            if os.name != 'nt':
                rc = os.spawnvp(os.P_WAIT, self.path, proc)
            else:
                rc = os.spawnv(os.P_WAIT, self.executable(self.path), proc)
            if rc == 127:
                raise JSolverError("Error: Error while trying to execute "+self.path)
        self.solution_time += clock()

        if not os.path.exists(tmpSol):
            raise JSolverError("Error: Error while executing "+self.path)
        lp.status, values = self.readsol(tmpSol)
        lp.assignVarsVals(values)
        if not self.keepFiles:
            try: os.remove(tmpLp)
            except: pass
            try: os.remove(tmpSol)
            except: pass
        return lp.status

    def readsol(self,filename):
        """Read a GLPK solution file"""
        with open(filename) as f:
            f.readline()
            rows = int(f.readline().split()[1])
            cols = int(f.readline().split()[1])
            f.readline()
            statusString = f.readline()[12:-1]
            glpkStatus = {
                "INTEGER OPTIMAL":LpStatusOptimal,
                "INTEGER NON-OPTIMAL":LpStatusOptimal,
                "OPTIMAL":LpStatusOptimal,
                "INFEASIBLE (FINAL)":LpStatusInfeasible,
                "INTEGER UNDEFINED":LpStatusUndefined,
                "UNBOUNDED":LpStatusUnbounded,
                "UNDEFINED":LpStatusUndefined,
                "INTEGER EMPTY":LpStatusInfeasible
            }
            #print "statusString ",statusString
            if statusString not in glpkStatus:
                raise JSolverError("Unknown status returned by GLPK")
            status = glpkStatus[statusString]
            isInteger = statusString in ["INTEGER NON-OPTIMAL","INTEGER OPTIMAL","INTEGER UNDEFINED"]
            values = {}
            for i in range(4): f.readline()
            for i in range(rows):
                line = f.readline().split()
                if len(line) ==2: f.readline()
            for i in range(3):
                f.readline()
            for i in range(cols):
                line = f.readline().split()
                name = line[1]
                if len(line) ==2: line = [0,0]+f.readline().split()
                if isInteger:
                    if line[2] == "*": value = int(float(line[3]))
                    else: value = float(line[2])
                else:
                    value = float(line[3])
                values[name] = value
        return status, values
GLPK = GLPK_CMD