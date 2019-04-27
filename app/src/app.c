/* Copyright 2019, Facundo Adrian Lucianna <facundolucianna@gmail.com>.
 *
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Date: 2019-04-06 */

/*==================[inclusions]=============================================*/

#include "camera_ov7670.h"      // Camera driver
#include "sapi.h"               // <= sAPI header

/*==================[macros and definitions]=================================*/
#define IMGBYTES  24768  // Numero de bytes en una imagen (QCIF) - Solo si se tienen en cuenta los bytes en posicion par o impar
#define IMGLINES  144    // Numero de lineas en una imagen (QCIF)
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){

    uint8_t photo[IMGBYTES];
    uint16_t lines[IMGLINES];

   /* ------------- INICIALIZACIONES ------------- */
   int8_t status = -1;   //status para ver el estado del sistema
   uint32_t count = 0;  // Contador para enviar los datos por puerto serie

   // Inicializar la placa
   boardConfig();

   // Inicializamos la camara
   status = ov7670Init();

   printf("Bienvenido al sistema de prueba de la camara OV7670\r\n");

   if(status < 0) {

     printf("Error de conexion\r\n");
     printf("Revise el PINOUT\r\n");

   } else {

     printf("OV7670 configurada\n");

   }

   // Sacamos una foto (para sacar mas de una foto, resetear el micro, cada vez que se quiere sacar una)

   // Prendemos el LED 1 para indicar al usuario que tiene 10 segundos para preparar la escena
   gpioWrite(LED1, ON);
   delay(10000);

   // Al apagarse el LED se idica que se saco la primera foto
   gpioWrite(LED1, OFF);

   // Sacamos la primera foto
   ov7670TakePhoto(0, 144, FALSE, &photo[0], &lines[0]);

   // Una vez obtenida la foto
   while (count < IMGBYTES) {

        printf("%d,", photo[count]);    //Enviamos por puerto serie la imagen
        photo[count] = 0;               //Limpiamos el buffer
        count = count + 1;

   }

   count = 0;
   printf("\r\n");

   //Enviamos por el numero de bytes por linea
   while (count < IMGLINES) {

        printf("%d,", lines[count]);
        lines[count] = 0;
        count = count + 1;

   }
   printf("\r\n");

   // Prendemos el LED 2 para indicar al usuario que viene la segunda foto
   gpioWrite(LED2, ON);
   delay(1000);

   // Al apagarse el LED se idica que se saco la primera foto
   gpioWrite(LED2, OFF);

   // Sacamos la segunda foto
   ov7670TakePhoto(0, 144, TRUE, &photo[0], &lines[0]);

   // Una vez obtenida la foto
   while (count < IMGBYTES) {

        printf("%d,", photo[count]);    //Enviamos por puerto serie la imagen
        photo[count] = 0;               //Limpiamos el buffer
        count = count + 1;

   }

   count = 0;
   printf("\r\n");

   //Enviamos por el numero de bytes por linea
   while (count < IMGLINES) {

        printf("%d,", lines[count]);
        lines[count] = 0;
        count = count + 1;

   }
   printf("\r\n");

   // Prendemos el LED 3 para indicar que el proceso ya termino
   gpioWrite(LED3, ON);

   while(1);
   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
	return 0 ;
}


/*==================[end of file]============================================*/
