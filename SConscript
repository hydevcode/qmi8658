
from building import *

cwd = GetCurrentDir()
src = Glob('src/*.c')
path = [cwd+"/include"]

group = DefineGroup('qmi8658', src, depend = ['PKG_USING_QMI8658'], CPPPATH = path)

Return('group')