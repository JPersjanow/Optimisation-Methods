class Matrix(dict):
    """ This is a dictionary based sparse matrix class
    """
    def __init__(self,rows,cols):
        """initialises the class by creating a matrix that will have the given
        rows and columns
        """
        self.rows = rows
        self.cols = cols
        self.rowdict = dict([(row, {}) for row in rows])
        self.coldict = dict([(col, {}) for col in cols])

    def add(self,row,col,item,colcheck = False, rowcheck = False):
        if (not(rowcheck and row not in self.rows)):
            if (not(colcheck and col not in self.cols)):
                dict.__setitem__(self,(row,col),item)
                self.rowdict[row][col] = item
                self.coldict[col][row] = item
            else:
                print(self.cols)
                raise RuntimeError("col %s is not in the matrix columns"%col)
        else:
            raise RuntimeError("row %s is not in the matrix rows"%row)

    def addcol(self,col,rowitems):
        """adds a column
        """
        if col in self.cols:
            for row,item in rowitems.items():
                self.add(row, col, item, colcheck = False)
        else:
            raise RuntimeError("col is not in the matrix columns")



    def get(self,k,d=0):
        return dict.get(self,k,d)

    def col_based_arrays(self):
        numEls = len(self)
        elemBase = []
        startsBase = []
        indBase = []
        lenBase = []
        for i,col in enumerate(self.cols):
            startsBase.append(len(elemBase))
            elemBase.extend(list(self.coldict[col].values()))
            indBase.extend(list(self.coldict[col].keys()))
            lenBase.append(len(elemBase) - startsBase[-1])
        startsBase.append(len(elemBase))
        return numEls, startsBase, lenBase, indBase, elemBase

if __name__ == "__main__":
    """ unit test
    """
    rows = list(range(10))
    cols = list(range(50,60))
    mat = Matrix(rows,cols)
    mat.add(1,52,"item")
    mat.add(2,54,"stuff")
    print(mat.col_based_arrays())