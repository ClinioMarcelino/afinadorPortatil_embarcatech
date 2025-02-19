# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/freitas/.pico-sdk/sdk/1.5.1/tools/pioasm"
  "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pioasm"
  "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/freitas/Projetos/Embarcatech/Prj_Afinador/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
