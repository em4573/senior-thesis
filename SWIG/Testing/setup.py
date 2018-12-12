from distutils.core import setup, Extension

example_module = Extension('_example', sources=['example_wrap.c', 'example.c'])
setup(name='example', ext_modules=[example_module], py_modules=["example"])