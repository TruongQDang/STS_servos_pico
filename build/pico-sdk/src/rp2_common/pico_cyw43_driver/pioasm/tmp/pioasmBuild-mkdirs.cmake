# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/truongdang/Documents/pico/pico-sdk/tools/pioasm"
  "/home/truongdang/Documents/graduation_project/pico_code/build/pioasm"
  "/home/truongdang/Documents/graduation_project/pico_code/build/pioasm-install"
  "/home/truongdang/Documents/graduation_project/pico_code/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/home/truongdang/Documents/graduation_project/pico_code/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
  "/home/truongdang/Documents/graduation_project/pico_code/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/home/truongdang/Documents/graduation_project/pico_code/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/truongdang/Documents/graduation_project/pico_code/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/truongdang/Documents/graduation_project/pico_code/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
