Codigo para calibración automática de camaras de acuerdo a:

http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html


Uso del programa:

Invocarlo utilizando el archivo que describe los parámetros del proceso de calibración. En este caso es el archivo "logitech_C920.xml". Al ejecutarlo los parámteros de calibración de la cámara se almacenan en el archivo out_camera_data.xml.

Se provee un programa de corrección de distorsion, que demuestra como leer el archivo out_camera_data.xml, y como invocar una rutina para corregir la distorsion en las imágenes.
