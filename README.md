# Librería de funciones para microcontrolador STM32
Este repositorio esta creado para la visualización del código del Trabajo de Fin de Grado para la carrera de Telecomunicaciones de Jesús Arribas López.
En el se pueden encontrar funciones destinadas al funcionamiento de varios sensores ambientales conectados a un microcontrolador STM32.
Estos sensores son los siguientes: WSEN-PADS, WSEN-HIDS, URM06 y AS7341.
#AS7341.c y AS741.h
Contiene las funciones destiandas al manejo del sensor AS7341
#WSENHIDS_humidity.c y WSENHIDS_humidity.h
Contiene las funciones destinadas al manejo del sensor WSEN-HIDS
#WSENPADS_pressure.c y WSEN_PADS_presure.h
Contiene las funciones destinadas al manejo del sensor WSEN-PADS
#main.c
Contiene el código que permite la lectura del pulso cuadrado del sensor URM06 y un código de ejemplo en el que se ejecuta el código de los cuatro sensores periódicamente.
