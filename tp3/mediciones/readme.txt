TODOS LOS ARCHIVOS DE MEDICIONES REALES TIENEN 5 VARIABLES POR SIMPLICIDAD Y EVITAR CAMBIAR EL SIMULINK TODO EL TIEMPO.
Las que no se usan, se setean en un valor arbitrario y constante.

-------------------------------------------------------------------------------------Observador.-----------------------------------------------------------------------------
Graficar mediciones reales de phi, theta, phi_punto y theta_punto (archivo mediciones_observador.mat) vs las mismas variables, pero estimadas por el observador, obtenidas con simulink (archivo simulaciones_observador.mat). En total resultan 4 gráficos, 1 para cada par: phi vs phi_hat, etc.

mediciones_observador.mat ---------- Contiene las mediciones reales de phi, theta, theta_punto, (d1,d2,d3 respectivamente) se debe calcular phi_punto para el informe.

-----------------------------------------------------Controlador por realimentación de estados (sin referencias).--------------------------------------------------------------

Se miden respuestas IMPULSIVAS (no step) del sistema, con amplitud 30°. Se deben comparar las mediciones vs simulaciones, tanto para las salidas como sus derivadas (phi, theta, phi_punto, theta_punto). En total resultan 4 gráficos, que provienen de los pares de salidas y sus derivadas.

mediciones_y_realimentacion.mat ---------- Contiene las mediciones de phi, theta, theta_punto (d1,d2,d3 respectivamente). Tomar la última medición, donde theta (d2) tiene el
pico más cercano a 30°.

-----------------------------------------------------Controlador por realimentación de estados (con referencias).--------------------------------------------------------------

Se miden las respuestas, generando una rutina de cambio de referencia para Phi, de tipo escalón y en +-30°. Se debe calcular phi_punto.

mediciones_y_con_ref.mat ---------- Contiene las medciones de phi,theta, theta_punto (d1,d2,d3 respectivamente).

-----------------------------------------------------Controlador por realimentación de estados (con acción integral).--------------------------------------------------------------

Se miden las respuestas, generando una rutina de cambio de referencia para Phi, de tipo escalón  y en +-30°.

mediciones_accion_integral.mat ---------- Contiene las medciones de phi,theta, theta_punto (d1,d2,d3 respectivamente). Se debe calcular phi_punto.

