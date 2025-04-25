
from building import *

cwd = GetCurrentDir()
src = []
path = [cwd+"/include"]

if GetDepend('PKG_USING_QMI8658') or GetDepend('RT_USING_I2C') :
    src += Glob('src/*.c')

group = DefineGroup('qmi8658', src, depend = ['PKG_USING_QMI8658'], CPPPATH = path)

Return('group')
