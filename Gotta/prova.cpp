#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#define dst 20
#include <stdio.h>
#include<cstdlib>
#include<vector>
#include<queue>
#include<map>
#include<math.h>

#define ii pair<int,int>

using namespace cv;
using namespace std;

Mat frame;
Mat edges;
Mat thr;
Mat drawing;
Mat lns=Mat::zeros( 240, 320, CV_8UC3 );

int erosion_elem = 0;
int erosion_size = 5;
int dilation_elem = 0;
int dilation_size = 2;
int const max_elem = 2;
int const max_kernel_size = 21;

ii iimake(int a, int b)
{
    return ii(make_pair(a, b));
}

int colorazione[3]={0,0,0};

void calcola_colore(int dist)
{
/*
    colorazione[0]=0;

colorazione[1]=255-dist;
if(colorazione[1]<0) colorazione[1]=0;
colorazione[2]=dist;
if(colorazione[2]>255) colorazione[2]=255;

*/
if(dist%20==0){
colorazione[0]=255;
colorazione[1]=255;
colorazione[2]=255;
}
else
{
colorazione[0]=0;
colorazione[1]=0;
colorazione[2]=0;
}

}


///VARIABILI DEL "ROBOT"-------------------
int soglianero=20;
int fino_a_dove=100;   ///FINO A DOVE: FIN DOVE CALCOLO LA DISTANZA, PIÙ CHE ALTRO FIN DOVE FACCIO TUTTO. ES: SE È 400, ALLORA
                        ///GUARDO (COSA GUARDO? quello che mi serve) fino a 400, appunto, pixel di distanza dal primo pixel
int distanza_isole=fino_a_dove-5; ///vedi sotto
int quanti_per_isola=3; ///QUANTI PER ISOLA: quanto è la distanza MASSIMA tra un pixel a una data distanza (distanza isole) di modo che si possano considerare
                        ///NON separati, cioè diciamo riunibili in uno stesso punto. Secondo la geometria, dovrebbe essere addirittura 1, non di più.



void Erosion( int, void* )
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( thr, thr, element );
}

/** @function Dilation */
void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( lns, lns, element );
}

void MyLine( Mat img, Point start, Point end )
{
  int thickness = 2;
  int lineType = 8;
  line( img,
    start,
    end,
    Scalar( 0, 0, 255 ),
    thickness,
    lineType );
}

void MyFilledCircle( Mat img, Point center )
{
 int thickness = -1;
 int lineType = 8;

 circle( img,
         center,
         3,
         Scalar( 0, 255, 0 ),
         thickness,
         lineType );
}

void MyCenterCircle( Mat img, Point center )
{
 int thickness = -1;
 int lineType = 8;

 circle( img,
         center,
         5,
         Scalar( 255, 0, 0 ),
         thickness,
         lineType );
}

void DetectLine(int level)
{
        //MyLine(frame,Point(20,460-dst*level),Point(620,460-dst*level));
        int a=0;
        int lastcoord;
        uchar intensity=thr.at<uchar>(460-dst*level,20);
        bool isline=false;
        if(intensity==0)
        {
            MyFilledCircle(frame,Point(20,460-dst*level));
            isline=true;
            lastcoord = 20;
            //a=50;
        }
        for(int i=21+a;i<=620;i++)
        {
            //cout << i<<endl;
            intensity=thr.at<uchar>(460-dst*level,i);
            if(isline==true&&(intensity==255||i==620))
            {
                isline=false;
                MyFilledCircle(frame,Point(i,460-dst*level));
                MyCenterCircle(frame,Point((i+lastcoord)/2,460-dst*level));
                //i+=100;
            }
            else if(isline==false&&intensity==0)
            {
                isline=true;
                MyFilledCircle(frame,Point(i,460-dst*level));
                lastcoord=i;
            }
        }

  //return pos;
}
queue<Point> coda;

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    namedWindow("Line",1);
    namedWindow("Original",1);
    namedWindow("Elab",1);
    /*
    namedWindow("Settings",0);
    createTrackbar( "Erosion --- Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Settings", &erosion_elem, max_elem);

    createTrackbar( "Erosion --- Kernel size:\n 2n +1", "Settings", &erosion_size, max_kernel_size);

    createTrackbar( "Dilation --- Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Settings", &dilation_elem, max_elem);

    createTrackbar( "Dilation --- Kernel size:\n 2n +1", "Settings", &dilation_size, max_kernel_size);

    */
    for(;;)
    {
        ///VARIABILI PER LA FUNZIONE--------------
        map<ii, int> distanze; ///la matrice dove il pixel in pos(x,y) è a distanza d dal primo nero trovato
        map<ii, bool> usati;  ///serve nella "contro"bfs, per vedere se ho già usato un punto

        int vett_quanti[20000]; ///ma vett_quanti è inutilissimo!
        vector<int>vettore_x[20000];
        vector<int>vettore_y[20000];///100mila vuol dire che possiamo arrivare fino alla distanza di 100mila pixel (esagerato)
        for(int contrl=0;contrl<5;contrl++){
        cap >> frame; // get a new frame from camera
        lns=Mat::zeros( 240, 320, CV_8UC3 );

        }
        //imshow("Line", frame);
        cvtColor(frame, edges, CV_BGR2GRAY);
        //imshow("Line", edges);
        threshold(edges,thr, 70, 255, THRESH_BINARY);

        //Erosion(0,0);
        ///Trovalinea

        int primox, primoy, lastx;
            ///NOTA PER TUTTO STO CESSO
            ///IL MAT::AT(A,B) FUNZIONA CHE A SONO LE RIGHE (QUINDI LA Y) E B LE COLONNE (LA X)
        for(int i=thr.rows-1; i>=0; i--)  ///i:righe
        {
            for(int j=thr.cols -1; j>=0; j--) ///j:colonne; CERCA IL PRIMO NERO A PARTIRE DA DESTRA
            {
                if ((int)thr.at<uchar>(i,j) == 0 )
                {
                    primox=j;
                    primoy=i;
                    j=-1;///esci dal for
                }
            }

            for(int j=0; j<thr.cols; j++) ///QUANDO HA TROVATO IL PRIMO NERO A DESTRA, LO CERCA DA SINISTRA
            {
                if ((int)thr.at<uchar>(i,j) == 0 )
                {
                    lastx=j;

                    //escimi il for
                    j=thr.cols+10;
                    i=-1;  ///QUANDO HA TROVATO IL NERO DX E IL NERO SX CHE STANNO PIÙ IN BASSO DI TUTI,
                            ///esce da tutti i for possibili, ha trovato le cordinate esatte del primo nero
                }
            }

        }

        primox=(primox+lastx)/2;
        MyFilledCircle(frame,Point(primox,primoy));
        ///BFS
        coda.push(Point(primox, primoy));
        distanze[iimake(primox,primoy)]=0;

        while(!coda.empty())
        {
            Point corrente = coda.front();
            coda.pop();
            int xcorrente=corrente.x;
            int ycorrente=corrente.y;

            for(int i=-1; i<=1; i++)
                for(int j=-1; j<=1; j++)   ///PER OGNI VICINO DEL PIXEL CORRENTE CONTROLLA SE LO DEVI PRENDERE
                {
                    int xprossimo = xcorrente+i;
                    int yprossimo = ycorrente+j;

                    if((xprossimo>=0 && xprossimo<thr.cols) &&
                        (yprossimo>=0 && yprossimo<thr.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                        &&
                        (distanze.count(iimake(xprossimo,yprossimo)) == 0 )&& ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                        ( (int)thr.at<uchar>(yprossimo,xprossimo) ==0) ///SE È UN NERO,
                        )
                    {

                        coda.push(Point(xprossimo,yprossimo));    ///metti il prossimo nodo nella coda
                        distanze[iimake(xprossimo,yprossimo)] = distanze[iimake(xcorrente,ycorrente)]+1; ///setta la distanza del prossimo a la tua +1
                        int distanza = distanze[iimake(xprossimo,yprossimo)]; ///e salvala nella variabile "distanza"
                        vett_quanti[distanza]++; ///di che c'è un altro punto a quella distanza

                        calcola_colore(distanza);  ///colora il punto sulla immagine
                        lns.at<Vec3b>(yprossimo,xprossimo) = (Vec3b){colorazione[0],colorazione[1],colorazione[2]};

                        vettore_x[distanza].push_back(xprossimo);
                        vettore_y[distanza].push_back(yprossimo);

                     } //endif
                } //endfor


        }//endwhile; fine bfs
        Dilation(0,0);
        imshow("Original",frame);
        imshow("Line", thr);
        imshow("Elab", lns);
        if(waitKey(30) >= 0) break;
        //waitKey(0);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
