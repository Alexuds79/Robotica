/*
 * ROBÓTICA - USAL. 2020. VISIÓN ARTIFICIAL
 *
 * ALEJANDRO MATEOS PEDRAZA		70969732N
 * JAVIER LÓPEZ SÁNCHEZ			70921610Y
 */

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h>

using namespace cv;
using namespace std;

Mat src, src_gray, img, marco, src_color;
int width;
int *cornersX = new int[8];
int *cornersY = new int[8];
int numCorners=0;

int thresh = 250;
int factorCercania = 70;
const char* corners_window = "Corners detected";

int cargaImagen(const char *);
void cornerHarris(int, void*);
void esquinasCriticas(int &, int &, int &);
int decide(int, int, int);


int main( int argc, char** argv ){
	int flag=0;
	int cornerMenorX, cornerMayorX, cornerMenorY;


	//Fijamos la posición inicial de robot de forma aleatoria
	srand(time(NULL));
	int photoIndex = rand()%5;

	switch(photoIndex){
		case 0:
			if (cargaImagen("RobotQR_Izquierda.png") == -1){
				return -1;
			}
		break;

		case 1:
			if (cargaImagen("RobotQR_Derecha.png") == -1){
				return -1;
			}
		break;

		case 2:
			if (cargaImagen("RobotQR_Lejos.png") == -1){
				return -1;
			}
		break;

		case 3:
			if (cargaImagen("RobotQR_IzquierdaLejos.png") == -1){
				return -1;
			}
		break;

		case 4:
			if (cargaImagen("RobotQR_DerechaLejos.png") == -1){
				return -1;
			}
		break;
	}

	//Bucle mientras no estemos centrados y cerca del objetivo
    while(flag != 22){

    	//ETAPA 1 | Filtrado: nos quedamos con el marco rojo
    	cvtColor(src, src_color, COLOR_BGR2HSV);
    	inRange(src_color, Scalar(170,100,100), Scalar(179,255,255), marco);
    	imshow("Frame", marco);

    	//ETAPA 2 | Algoritmo de esquinas de Harris
    	cornerHarris(0, 0);

    	//ETAPA 3 | Calculamos las esquinas críticas
    	esquinasCriticas(cornerMenorX, cornerMayorX, cornerMenorY);

    	//ETAPA 4 | El robot decide su siguiente movimiento
    	flag = decide(cornerMenorX, width-cornerMayorX, cornerMenorY);

    	waitKey();

    	//Mostramos la siguiente imagen en función de la decisión
    	switch(flag){
    		case 10: //Lejos y debo desplazarme a derecha para centrarme
    		case 11: //Lejos y debo desplazarme a izquierda para centrarme
    			if (cargaImagen("RobotQR_Lejos.png") == -1){
    				return -1;
    			}
    		break;

    		case 12: //Lejos y centrado
    			if (cargaImagen("RobotQR.png") == -1){
    				return -1;
    			}
    		break;

    		case 20: //Cerca y debo desplazarme a derecha para centrarme
    		case 21: //Cerca y debo desplazarme a izquierda para centrarme
    			if (cargaImagen("RobotQR.png") == -1){
    				return -1;
    			}
    		break;
    	}
    }

    return 0;
}

int cargaImagen(const char *imgName){
	img = imread(imgName);

	if (img.empty()){
	   cout << "ERROR. No se ha podido tomar la imagen!\n" << endl;
	   return -1;
	}

	resize(img, src, src.size(), 0.2, 0.2, INTER_LINEAR); //Para que la imagen no se despliegue demasiado grande
	width = src.cols; //Calculamos el ancho total de la imagen
	imshow("Original", src);

	return 0;
}

//Fuente: https://docs.opencv.org/3.4/d4/d7d/tutorial_harris_detector.html
void cornerHarris(int, void*){
	numCorners=0;
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    Mat dst = Mat::zeros( src.size(), CV_32FC1 );

    cornerHarris(marco, dst, blockSize, apertureSize, k);
    Mat dst_norm, dst_norm_scaled;

    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );

    for(int i = 0; i < dst_norm.rows ; i++){
        for(int j = 0; j < dst_norm.cols; j++){
            if((int) dst_norm.at<float>(i,j) > thresh){
                circle(dst_norm_scaled, Point(j,i), 5,  Scalar(0,0,255), 2, 9, 0);

                //Almacenamos las esquinas
                cornersX[numCorners]=j; //j: valores en el eje x de las esquinas
                cornersY[numCorners]=i; //i: valores en el eje y de las esquinas
                numCorners++;
            }
        }
    }

    namedWindow(corners_window);
    imshow(corners_window, dst_norm_scaled);
}

void esquinasCriticas(int &cornerMenorX, int &cornerMayorX, int &cornerMenorY){

	//Calculamos el mayor y menor valor en x de todas las esquinas
	//Estos valores nos servirán para saber si el objetivo está centrado
	cornerMenorX = cornersX[0];
	cornerMayorX = cornersX[0];

	for (int i=1; i<numCorners; i++){
		if (cornersX[i] > cornerMayorX) cornerMayorX = cornersX[i];
		if (cornersX[i] < cornerMenorX) cornerMenorX = cornersX[i];
	}

	cornerMenorY = cornersY[0];

	//Calculamos el menor valor en y de todas las esquinas
	//Este valor nos servirá para saber si estamos cerca del objetivo
	for (int i=1; i<numCorners; i++){
			if (cornersY[i] < cornerMenorY) cornerMenorY = cornersY[i];
		}
}

int decide(int distanciaIzquierda, int distanciaDerecha, int distanciaVertical){
	if (distanciaVertical > factorCercania){
			cout << "QR-ROBOT > Estoy lejos del objetivo. Debo acercarme..." << endl;

			if (distanciaIzquierda > distanciaDerecha){
				cout << "QR-ROBOT > Tengo que desplazarme a la derecha..." << endl;
				return 10;
			}

			else if (distanciaIzquierda < distanciaDerecha){
				cout << "QR-ROBOT > Tengo que desplazarme a la izquierda..." << endl;
				return 11;
			}

			else{
				cout << "QR-ROBOT > Estoy centrado..." << endl;
				return 12;
			}
	}

	else{
		cout << "QR-ROBOT > Estoy cerca del objetivo..." << endl;

		if (distanciaIzquierda > distanciaDerecha){
			cout << "QR-ROBOT > Tengo que desplazarme a la derecha..." << endl;
			return 20;
		}

		else if (distanciaIzquierda < distanciaDerecha){
			cout << "QR-ROBOT > Tengo que desplazarme a la izquierda..." << endl;
			return 21;
		}

		else{
			cout << "QR-ROBOT > Estoy centrado..." << endl;
			cout << "QR-ROBOT > LEYENDO CÓDIGO QR..." << endl;
			return 22;
		}
	}
}
