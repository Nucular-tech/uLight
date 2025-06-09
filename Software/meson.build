# meson.build
# Debug build: compile -C Debug
# Debug prepare: setup Debug --cross-file meson.cross --reconfigure
# Release build: compile -C Release
# Release prepare: setup Release --cross-file meson.cross --reconfigure --buildtype=release --optimization=2
# DO NOT USE GCC v13 !! it is bugged
project('Nucular uLight', 'c', 'cpp',
  version: '0.6.12',
  default_options: [
    'buildtype=debug',    # Set the build type
    'warning_level=2',    # Adjust warning level (optional)
    'optimization=g',      # Optimization level (optional)
    'c_std=gnu2x,c11',
    'cpp_std=gnu++20,c++11'
  ],
  meson_version: '>= 1.4.0'
)

# Set up the compiler flags for Cortex-M4 with FPU
lang_ru = '-DRUSSIAN'
lang_en = '-DENGLISH'
defines = [
	'STM32F10X_MD',
	'HSE_VALUE=8000000'
]
if get_option('buildtype') == 'debug'
    defines += [
    'DEBUG', 
    'TRACE',
    'OS_USE_TRACE_SEMIHOSTING_DEBUG',
    ]  # Enable debug symbols
endif

# Set up the compiler flags for Cortex-M4 with FPU
cpuflags = [
	'-mcpu=cortex-m3',              # Specify the CPU architecture
	'-mthumb',
]

# GCC flags for C and CPP
cflags = cpuflags + [
	'-ffunction-sections',          # Optimize for function sections (optional)
	'-fdata-sections',              # Optimize for data sections (optional)
	'-ffreestanding',				# no OS
	'-fsingle-precision-constant',	# force float everywhere 
	'-fno-move-loop-invariants',
	'-fms-extensions',				# Allow Microsoft's version of anonymous unions and struct
	'-fno-math-errno',				# dont use math errno variable
	'-gdwarf-4',					# dwarf 4 debug info
	'-fanalyzer',					# run code analysis 
	'-g3',                          # Generate debug information (optional)
	'-Wdouble-promotion',
	'-Wimplicit-fallthrough',
]

# Linker flags
linker_flags = cflags + [
	'-nostdlib',
	'-nostartfiles',
	'-Xlinker',
	'--gc-sections',
	'--specs=nano.specs',
	'--specs=nosys.specs',
	'-Wl,--print-memory-usage'
]
foreach def : defines
    cflags += '-D' + def
endforeach

# Define standard libraries
standard_libs = [
    'm',   # Math library
    'c_nano'  # Lightweight C library 
]
# convert libraries to library includes for linker
foreach lib : standard_libs
    linker_flags += ['-l' + lib]
endforeach

# Define your executable (assuming main.c is your entry point)
#executable('main', 'main.c', install: true)

# Optionally, you can add more source files or libraries
# Include subdirectories (FreeRTOS, LevCan, system, user, etc.)
global_sources = files(
	#LEVCAN
	'../../LEVCAN/source/levcan_address.c',
	'../../LEVCAN/source/levcan_events.c',
	'../../LEVCAN/source/levcan_fileclient.c',
	'../../LEVCAN/source/levcan_paramcommon.c',
	'../../LEVCAN/source/levcan_paramserver.c',
	'../../LEVCAN/source/levcan.c',
	# Code Tools
	'CTools-STM32/armutils.c',
	'CTools-STM32/datetime_formatter.c',
	'CTools-STM32/filters.c',
	'CTools-STM32/flash.c',
	'CTools-STM32/memcchr.c',
	'CTools-STM32/storage.c',
	'CTools-STM32/xformathandler.c',
	'CTools-STM32/xformatc.c',
)
# Include paths
inc_dir=[
	'../../LEVCAN/source/',
	'CTools-STM32/'
]
# Include additional folders (with own meson.build files)
subdir('system')
subdir('user')

# Linker files
subdir('ldscripts')
foreach script : ldscript_files
    linker_flags += '-T' + script.full_path()
endforeach

# Pass the version information to GCC as a preprocessor definition
project_version = meson.project_version()
version_date = ['-DMESON_VERSION="' + project_version + '"']

# Add flags for both C and C++
add_project_arguments(version_date, language: ['c', 'cpp'])
add_project_arguments(cflags, language: ['c', 'cpp'])
# Add linker flags
add_project_link_arguments(linker_flags, language: ['c', 'cpp'])
# Define the executable
elfs = []
if get_option('buildtype') == 'debug' 
	elf_name = 'NuLight'
	elfs += executable(
		elf_name + '.elf', 
		global_sources, 
		include_directories : inc_dir,
		cpp_args : lang_en,
		c_args : lang_en,
		link_args : ['-Wl,-Map=' + elf_name + '.map'])
else
	foreach buildindex : [['NuLightEN', lang_en], ['NuLightRU', lang_ru]]
		# [0] elf name, [1] language
		elfs += executable(
			buildindex[0] + '.elf', 
			global_sources, 
			include_directories : inc_dir,
			cpp_args : buildindex[1],
			c_args : buildindex[1],
			link_args : ['-Wl,-Map=' + buildindex[0] + '.map'])
	endforeach
endif	

build_dir = meson.project_build_root()
hex_files = {}
# post build  convert ELF to Intel HEX
foreach elf : elfs
	# Create a HEX file from the ELF file
	hex_name = elf.name().replace('.elf','')
	run_command('mkdir', '-p', join_paths(build_dir, hex_name), check: true)
	hex_files += {hex_name : custom_target(
	    hex_name,               # Target name
	    input: elf,             # Input ELF file
	    output: hex_name,		# Output HEX file
	    command: ['objcopy', '-O', 'ihex', '@INPUT@', hex_name + '/NuLight.hex'], # Conversion command
	    depends: elf,					# Ensure ELF is built first
		build_by_default: true                    
	)}
endforeach
