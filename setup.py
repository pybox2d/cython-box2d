import os
from distutils.core import (setup, Extension)
import glob
from Cython.Build import cythonize
from Cython.Distutils import build_ext


box2d_source_top = os.path.join('pybox2d', )
box2d_dirs = [os.path.join(box2d_source_top, path)
              for path in ('Box2D',
                           os.path.join('Box2D', 'Common'),
                           os.path.join('Box2D', 'Collision'),
                           os.path.join('Box2D', 'Collision', 'Shapes'),
                           os.path.join('Box2D', 'Dynamics'),
                           os.path.join('Box2D', 'Dynamics', 'Contacts'),
                           os.path.join('Box2D', 'Dynamics', 'Joints'),
                           os.path.join('Box2D', 'Rope'),
                           )
              ]

cpp_source_files = sum((glob.glob(os.path.join(path, '*.cpp'))
                        for path in box2d_dirs),
                       [])

ext = Extension(
    "pybox2d",
    ["pybox2d/main.pyx"] + cpp_source_files,
    language="c++",
    include_dirs=['.', box2d_source_top] + box2d_dirs,
    libraries=[],
    # extra_compile_args=['-Wno-unneeded-internal-declaration',
    #                     '-Wno-unused-function'],
    # extra_link_args=['-v'],
)


setup(name='pybox2d',
      ext_modules=[ext],
      cmdclass={'build_ext': build_ext},
      )
