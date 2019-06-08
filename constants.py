VERSION = '1.6.11'
EPS = 1e-7

# variable categories
LpContinuous = "Continuous"
LpInteger = "Integer"
LpBinary = "Binary"
LpCategories = {LpContinuous: "Continuous", LpInteger: "Integer",
                LpBinary: "Binary"}

# objective sense
LpMinimize = 1
LpMaximize = -1
LpSenses = {LpMaximize:"Maximize", LpMinimize:"Minimize"}

# problem status
LpStatusNotSolved = 0
LpStatusOptimal = 1
LpStatusInfeasible = -1
LpStatusUnbounded = -2
LpStatusUndefined = -3
LpStatus = { LpStatusNotSolved:"Not Solved",
             LpStatusOptimal:"Optimal",
             LpStatusInfeasible:"Infeasible",
             LpStatusUnbounded:"Unbounded",
             LpStatusUndefined:"Undefined",
             }

# constraint sense
LpConstraintLE = -1
LpConstraintEQ = 0
LpConstraintGE = 1
LpConstraintSenses = {LpConstraintEQ:"=", LpConstraintLE:"<=", LpConstraintGE:">="}

# LP line size
LpCplexLPLineSize = 78

def isiterable(obj):
    try: obj=iter(obj)
    except: return False
    else: return True

class JError(Exception):
    """
    Pulp Exception Class
    """
    pass