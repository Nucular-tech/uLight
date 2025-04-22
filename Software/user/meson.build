inc_dir += include_directories('.')
inc_dir += include_directories('Peripherial')

# Peripherial sources
peripherial_sources = files(
  'Peripherial/adc.c',
  'Peripherial/can_hal.c',
  'Peripherial/gpio_hal.c',
  'Peripherial/pwm.c',
)

# Data sources
data_sources = files(
  'Data/hwconfig.c',
  'Data/parameters.c',
)

user_sources = files(
  'logic.c',
  'main.c',
  'network.c',
)
if get_option('buildtype') == 'debug'
  user_sources += files(
  )
endif

# Append to the global source list
global_sources += user_sources + peripherial_sources + data_sources 
