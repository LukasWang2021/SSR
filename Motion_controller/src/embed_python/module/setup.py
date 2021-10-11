from distutils.core import setup, Extension
# from sysconfig import *
import distutils.sysconfig
import sys

distutils.sysconfig.get_config_var('EXT_SUFFIX')
distutils.sysconfig._config_vars['EXT_SUFFIX'] = '.cpython-39-aarch64-linux-gnu.so'

PLATFORM = "zcu102"
CROSS_ROOT = "/usr/local/crosstool/" + PLATFORM
GCC_PATH_ROOT = "/usr/local/aarch64-linux-gnu"
USR_LIB_DIR = CROSS_ROOT + "/usr/lib/aarch64-linux-gnu"
USR_LOCAL_LIB_DIR = CROSS_ROOT + "/usr/local/lib/aarch64-linux-gnu"

LIBRARY_DIRS = [USR_LIB_DIR, USR_LOCAL_LIB_DIR]
LIBRARIES = ['embed_python']
INCLUDE_DIRS = [CROSS_ROOT + "/usr/include", 
              CROSS_ROOT + "/usr/local/include", 
              CROSS_ROOT + "/usr/include/libxml2",
              CROSS_ROOT + "/usr/include/aarch64-linux-gnu",
              CROSS_ROOT + "/usr/local/include/python3.9/internal", 
              CROSS_ROOT + "/usr/local/include/python3.9/cpython", 
              CROSS_ROOT + "/usr/local/include/python3.9", 
              GCC_PATH_ROOT + "/aarch64-linux-gnu/include/c++/7.2.1",
              GCC_PATH_ROOT + "/aarch64-linux-gnu/include/c++/7.2.1/aarch64-linux-gnu",
              "../../common", 
              "../../xml_help/include",
              "../../yaml_help/include", 
              "../../sem_help/include", 
              "../../thread_help/include",  
              "../../log_manager/include", 
              "../../error_queue/include",
              "../../algorithm_base/include",
              "../../io_1000/include",
              "../../rtm_spi/include",
              "../../nvram/include",
              "../../joint_constraint/include",
              "../../kinematics_alg/include",
              "../../dynamic_alg/include",
              "../../basic_alg/include",
              "../../transformation/include",
              "../../trajectory_planner/include",
              "../../system_model_manager/include",
              "../../core_comm/include",
              "../../servo_comm/include",
              "../../base_device/include",
              "../../axis/include",
              "../../coordinate_manager/include",
              "../../tool_manager/include",
              "../../group/include",
              "../../motion_control/include",]


controller_module = Extension('controller',
                    define_macros = [('MAJOR_VERSION', '1'),('MINOR_VERSION', '0')],
                    include_dirs = INCLUDE_DIRS + ['..//include', ],
                    library_dirs = LIBRARY_DIRS + ['../../../install/lib'],
                    libraries = LIBRARIES,
                    sources = ['controllermodule.cpp'])

group_module = Extension('group',
                    define_macros = [('MAJOR_VERSION', '1'),('MINOR_VERSION', '0')],
                    include_dirs = INCLUDE_DIRS + ['..//include', ],
                    library_dirs = LIBRARY_DIRS + ['../../../install/lib'],
                    libraries = LIBRARIES,
                    sources = ['groupmodule.cpp'])

registermodule = Extension('register',
                    define_macros = [('MAJOR_VERSION', '1'),('MINOR_VERSION', '0')],
                    include_dirs = INCLUDE_DIRS + ['..//include', ],
                    library_dirs = LIBRARY_DIRS + ['../../../install/lib'],
                    libraries = LIBRARIES,
                    sources = ['registermodule.cpp'])

setup(name = 'controller',
       version = '1.0',
       description = 'This pakage supply the base function to controller option.',
       author = 'wuym',
       author_email = 'youming.wu@rtimeman.com',
       url = 'https://docs.python.org/extending/building',
       long_description = '''
       Axis,group,hardware devices option interfaces.
       ''',
       platforms = 'aarch64-linux-gnu',
       ext_modules = [controller_module, group_module, registermodule])

# export EXT_SUFFIX=.cpython-39-aarch64-linux-gnu.so

