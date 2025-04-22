# List of FreeRTOS sources
inc_dir += include_directories('.')
inc_dir += include_directories('cmsis')
inc_dir += include_directories('cortexm')
inc_dir += include_directories('diag')
system_sources = [
  'cmsis/system_stm32f10x.c',
  'cmsis/vectors_stm32f10x.c',
  'cortexm/_initialize_hardware.c',
  'cortexm/_reset_hardware.c',
  'cortexm/exception_handlers.c',
  'diag/trace_impl.c',
  'diag/Trace.c',
  'newlib/_exit.c',
  'newlib/_sbrk.c',
  'newlib/_startup.c',
  'newlib/_syscalls.c',
  'newlib/assert.c',
]

# Append to the global source list
global_sources += files(system_sources)