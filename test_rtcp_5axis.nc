( ===================================================================== )
( PROGRAMA DE PRUEBA RTCP 5 EJES - grblHAL Simulator                   )
( ===================================================================== )
( Autor: Generado para verificacion de cinematica RTCP v17.1            )
( Maquina: Mesa AC con Pivot Z=150mm                                    )
( Config: $640=0, $641=0, $642=150, $643=0, $644=0                     )
(                                                                       )
( FASES:                                                                )
(   1. Domo hemisferico - fresado 3+2 en multiples inclinaciones        )
(   2. Estrella a 45 grados - patron 2D inclinado                       )
(   3. Cono helicoidal - 5 ejes simultaneos                             )
(   4. Perfil de alabe - contorneado 5 ejes continuo                    )
(   5. Taladrado indexado - posiciones angulares discretas               )
(   6. Flor 5 ejes - patron circular con A y C continuos                )
( ===================================================================== )

( --- INICIALIZACION --- )
G90 G21 G17          ( Absoluto, milimetros, plano XY )
G94                  ( Feed rate en mm/min )
M451                 ( RTCP ON )
F300                 ( Feed rate inicial )
G0 X0 Y0 Z10 A0 C0  ( Home seguro )

( ===================================================================== )
( FASE 1: DOMO HEMISFERICO - Fresado 3+2 en capas inclinadas           )
( Simula el mecanizado de una superficie curva usando multiples         )
( orientaciones fijas de herramienta                                    )
( ===================================================================== )

( --- Capa 1: A=0 - fresado horizontal --- )
G0 Z5 A0 C0
G0 X-30 Y-30
G1 Z-2 F200

G1 X30 Y-30 F300
G1 X30 Y-20
G1 X-30 Y-20
G1 X-30 Y-10
G1 X30 Y-10
G1 X30 Y0
G1 X-30 Y0
G1 X-30 Y10
G1 X30 Y10
G1 X30 Y20
G1 X-30 Y20
G1 X-30 Y30
G1 X30 Y30

G0 Z5

( --- Capa 2: A=15, C=0 - inclinacion leve --- )
G0 A15 C0
G0 X-25 Y-25
G1 Z-3 F150

G1 X25 Y-25 F250
G1 X25 Y-15
G1 X-25 Y-15
G1 X-25 Y-5
G1 X25 Y-5
G1 X25 Y5
G1 X-25 Y5
G1 X-25 Y15
G1 X25 Y15
G1 X25 Y25
G1 X-25 Y25

G0 Z5

( --- Capa 3: A=30, C=45 - inclinacion moderada girada --- )
G0 A30 C45
G0 X-20 Y-20
G1 Z-4 F150

G1 X20 Y-20 F200
G1 X20 Y-10
G1 X-20 Y-10
G1 X-20 Y0
G1 X20 Y0
G1 X20 Y10
G1 X-20 Y10
G1 X-20 Y20
G1 X20 Y20

G0 Z5

( --- Capa 4: A=45, C=90 - inclinacion fuerte girada 90 --- )
G0 A45 C90
G0 X-15 Y-15
G1 Z-3 F100

G1 X15 Y-15 F180
G1 X15 Y-5
G1 X-15 Y-5
G1 X-15 Y5
G1 X15 Y5
G1 X15 Y15
G1 X-15 Y15

G0 Z5

( Retorno a posicion neutra )
G0 A0 C0
G0 X0 Y0 Z10

( ===================================================================== )
( FASE 2: ESTRELLA a 45 grados - Patron geometrico inclinado           )
( Prueba movimientos XY con A fijo y C fijo                            )
( ===================================================================== )

G0 A45 C0
G0 X0 Y0 Z5
G1 Z-2 F150

( Estrella de 5 puntas )
G1 X0    Y30   F300
G1 X-28.5  Y-9.3
G1 X17.6  Y24.3
G1 X-17.6 Y24.3
G1 X28.5  Y-9.3
G1 X0    Y30

G0 Z5

( Misma estrella rotada C=72 )
G0 C72
G0 X0 Y0
G1 Z-2 F150

G1 X0    Y30   F300
G1 X-28.5  Y-9.3
G1 X17.6  Y24.3
G1 X-17.6 Y24.3
G1 X28.5  Y-9.3
G1 X0    Y30

G0 Z5

( Misma estrella rotada C=144 )
G0 C144
G0 X0 Y0
G1 Z-2 F150

G1 X0    Y30   F300
G1 X-28.5  Y-9.3
G1 X17.6  Y24.3
G1 X-17.6 Y24.3
G1 X28.5  Y-9.3
G1 X0    Y30

G0 Z5

G0 A0 C0
G0 X0 Y0 Z10

( ===================================================================== )
( FASE 3: CONO HELICOIDAL - 5 ejes simultaneos                        )
( El TCP traza una espiral descendente mientras A se inclina            )
( progresivamente. Esto prueba interpolacion simultanea en              )
( todos los ejes.                                                       )
( ===================================================================== )

G0 X20 Y0 Z0 A0 C0
G1 Z-1 F100

( Espiral descendente con inclinacion creciente )
( Vuelta 1: R=20, A=0->10 )
G1 X17.32 Y10    Z-1.5  A2   C30   F200
G1 X10    Y17.32 Z-2    A4   C60
G1 X0     Y20    Z-2.5  A6   C90
G1 X-10   Y17.32 Z-3    A8   C120
G1 X-17.32 Y10   Z-3.5  A10  C150
G1 X-20   Y0     Z-4    A12  C180
G1 X-17.32 Y-10  Z-4.5  A14  C210
G1 X-10   Y-17.32 Z-5   A16  C240
G1 X0     Y-20   Z-5.5  A18  C270
G1 X10    Y-17.32 Z-6   A20  C300
G1 X17.32 Y-10   Z-6.5  A22  C330
G1 X20    Y0     Z-7    A24  C360

( Vuelta 2: R=15, A=24->45 )
G1 X12.99 Y7.5   Z-7.5  A27  C390
G1 X7.5   Y12.99 Z-8    A30  C420
G1 X0     Y15    Z-8.5  A32  C450
G1 X-7.5  Y12.99 Z-9    A34  C480
G1 X-12.99 Y7.5  Z-9.5  A36  C510
G1 X-15   Y0     Z-10   A38  C540
G1 X-12.99 Y-7.5 Z-10.5 A40  C570
G1 X-7.5  Y-12.99 Z-11  A42  C600
G1 X0     Y-15   Z-11.5 A43  C630
G1 X7.5   Y-12.99 Z-12  A44  C660
G1 X12.99 Y-7.5  Z-12.5 A44.5 C690
G1 X15    Y0     Z-13   A45  C720

( Vuelta 3: R=10, A=45 constante, C continua )
G1 X8.66  Y5     Z-13.5 A45  C750
G1 X5     Y8.66  Z-14   A45  C780
G1 X0     Y10    Z-14.5 A45  C810
G1 X-5    Y8.66  Z-15   A45  C840
G1 X-8.66 Y5     Z-15.5 A45  C870
G1 X-10   Y0     Z-16   A45  C900
G1 X-8.66 Y-5    Z-16.5 A45  C930
G1 X-5    Y-8.66 Z-17   A45  C960
G1 X0     Y-10   Z-17.5 A45  C990
G1 X5     Y-8.66 Z-18   A45  C1020
G1 X8.66  Y-5    Z-18.5 A45  C1050
G1 X10    Y0     Z-19   A45  C1080

G0 Z5
G0 A0 C0
G0 X0 Y0 Z10

( ===================================================================== )
( FASE 4: PERFIL DE ALABE - Contorneado 5 ejes continuo               )
( Simula el mecanizado de un perfil aerodinamico de turbina.           )
( La herramienta sigue el contorno manteniendose normal a la           )
( superficie, variando A y C constantemente.                            )
( ===================================================================== )

G0 X-40 Y0 Z0 A0 C0

( Lado presion del alabe - A varia para seguir curvatura )
G1 X-40 Y0  Z-5  A-10 C0   F150
G1 X-35 Y3  Z-5  A-8  C5
G1 X-30 Y5  Z-5  A-5  C8
G1 X-25 Y7  Z-5  A-3  C10
G1 X-20 Y8  Z-5  A0   C12
G1 X-15 Y9  Z-5  A3   C13
G1 X-10 Y9.5 Z-5 A5   C14
G1 X-5  Y9.8 Z-5 A7   C14.5
G1 X0   Y10  Z-5 A8   C15
G1 X5   Y9.8 Z-5 A10  C14.5
G1 X10  Y9   Z-5 A12  C13
G1 X15  Y8   Z-5 A15  C11
G1 X20  Y6   Z-5 A18  C8
G1 X25  Y4   Z-5 A22  C5
G1 X30  Y2   Z-5 A25  C2
G1 X35  Y0.5 Z-5 A28  C0
G1 X40  Y0   Z-5 A30  C0

( Borde de ataque - transicion con C girando )
G1 X40 Y0   Z-5 A30  C0   F120
G1 X42 Y-1  Z-4 A25  C-5
G1 X43 Y-2  Z-3 A20  C-10
G1 X43 Y-3  Z-2 A15  C-15
G1 X42 Y-4  Z-1 A10  C-18

( Lado succion del alabe - regreso con diferente inclinacion )
G1 X40  Y-4   Z-3 A8   C-20  F150
G1 X35  Y-5   Z-3 A5   C-18
G1 X30  Y-5.5 Z-3 A3   C-15
G1 X25  Y-6   Z-3 A0   C-12
G1 X20  Y-6   Z-3 A-2  C-10
G1 X15  Y-5.5 Z-3 A-4  C-8
G1 X10  Y-5   Z-3 A-6  C-6
G1 X5   Y-4   Z-3 A-8  C-4
G1 X0   Y-3   Z-3 A-9  C-2
G1 X-5  Y-2.5 Z-3 A-10 C0
G1 X-10 Y-2   Z-3 A-10 C2
G1 X-15 Y-1.5 Z-3 A-10 C3
G1 X-20 Y-1   Z-3 A-10 C4
G1 X-25 Y-0.5 Z-3 A-10 C4.5
G1 X-30 Y-0.3 Z-3 A-10 C4
G1 X-35 Y-0.1 Z-3 A-10 C2
G1 X-40 Y0    Z-3 A-10 C0

G0 Z10
G0 A0 C0

( ===================================================================== )
( FASE 5: TALADRADO INDEXADO - Posiciones angulares discretas          )
( Simula taladrado de agujeros en una pieza cilindrica,                )
( rotando C cada 60 grados con A=30 fijo.                              )
( Prueba posicionamiento 3+2 preciso.                                   )
( ===================================================================== )

( Agujero 1: C=0 )
G0 X0 Y0 Z5 A30 C0
G0 X25 Y0
G1 Z-8 F80
G0 Z5

( Agujero 2: C=60 )
G0 C60
G0 X25 Y0
G1 Z-8 F80
G0 Z5

( Agujero 3: C=120 )
G0 C120
G0 X25 Y0
G1 Z-8 F80
G0 Z5

( Agujero 4: C=180 )
G0 C180
G0 X25 Y0
G1 Z-8 F80
G0 Z5

( Agujero 5: C=240 )
G0 C240
G0 X25 Y0
G1 Z-8 F80
G0 Z5

( Agujero 6: C=300 )
G0 C300
G0 X25 Y0
G1 Z-8 F80
G0 Z5

G0 A0 C0
G0 X0 Y0 Z10

( ===================================================================== )
( FASE 6: FLOR 5 EJES - Patron artistico con A y C continuos          )
( Patron de petalos donde el TCP traza una flor en XY mientras         )
( A oscila como onda senoidal y C gira continuamente.                   )
( MAXIMO STRESS de la cinematica RTCP.                                  )
( ===================================================================== )

G0 X20 Y0 Z0 A0 C0
G1 Z-1 F100

( Petalo 1: 0-72 grados )
G1 X19   Y6.2  Z-1   A5    C10   F180
G1 X16.2 Y11.8 Z-1.5 A12   C20
G1 X11.8 Y16.2 Z-2   A20   C30
G1 X6.2  Y19   Z-2   A28   C40
G1 X0    Y20   Z-2.5 A35   C50
G1 X-4   Y18   Z-2   A30   C55
G1 X-2   Y14   Z-1.5 A22   C60
G1 X2    Y10   Z-1   A12   C65
G1 X6.2  Y6.2  Z-0.5 A5    C72

( Petalo 2: 72-144 grados )
G1 X3    Y9    Z-1   A10   C82
G1 X-2   Y14   Z-1.5 A18   C92
G1 X-8   Y17   Z-2   A26   C102
G1 X-14  Y18   Z-2   A33   C112
G1 X-19  Y6.2  Z-2.5 A40   C122
G1 X-17  Y2    Z-2   A34   C127
G1 X-13  Y-2   Z-1.5 A25   C132
G1 X-8   Y-4   Z-1   A15   C138
G1 X-3   Y-2   Z-0.5 A5    C144

( Petalo 3: 144-216 grados )
G1 X-6   Y-6   Z-1   A10   C154
G1 X-11  Y-10  Z-1.5 A18   C164
G1 X-16  Y-12  Z-2   A26   C174
G1 X-19  Y-10  Z-2   A33   C184
G1 X-19  Y-6.2 Z-2.5 A40   C194
G1 X-16  Y-8   Z-2   A34   C199
G1 X-12  Y-12  Z-1.5 A25   C204
G1 X-7   Y-14  Z-1   A15   C210
G1 X-2   Y-12  Z-0.5 A5    C216

( Petalo 4: 216-288 grados )
G1 X2    Y-14  Z-1   A10   C226
G1 X6    Y-17  Z-1.5 A18   C236
G1 X11   Y-18  Z-2   A26   C246
G1 X15   Y-17  Z-2   A33   C256
G1 X17.6 Y-12  Z-2.5 A40   C266
G1 X16   Y-8   Z-2   A34   C271
G1 X13   Y-4   Z-1.5 A25   C276
G1 X9    Y-1   Z-1   A15   C282
G1 X5    Y1    Z-0.5 A5    C288

( Petalo 5: 288-360 grados - cierre )
G1 X9    Y-2   Z-1   A10   C298
G1 X14   Y-6   Z-1.5 A18   C308
G1 X18   Y-8   Z-2   A26   C318
G1 X20   Y-5   Z-2   A33   C328
G1 X20   Y0    Z-2.5 A40   C338
G1 X19   Y3    Z-2   A30   C345
G1 X17   Y4    Z-1.5 A20   C350
G1 X14   Y3    Z-1   A10   C355
G1 X10   Y1    Z-0.5 A5    C358

( Cierre al inicio )
G1 X20   Y0    Z0    A0    C360  F120

G0 Z10
G0 A0 C0

( ===================================================================== )
( FINALIZACION                                                          )
( ===================================================================== )
G0 X0 Y0 Z20 A0 C0
M451                 ( RTCP OFF )
M2                   ( Fin de programa )
