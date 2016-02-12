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


///---------------------funzioni  e variabili di utils

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

///-------------------------------funzioni e variabili del programma

struct Mypoint{
int x;
int y;
int dist;

};


ii iimake(int a, int b)
{
    return ii(make_pair(a, b));
}

int colorazione[3]={0,0,0};


void calcola_colore(int dist)
{

int qualecolore = (dist/250)%3;
int intensita=(dist%250);

for(int i=0; i<3; i++)
    colorazione[i]=0;

colorazione[qualecolore]=intensita;

}



///VARIABILI DEL "ROBOT"-------------------
int soglianero=120;
int fino_a_dove=100;   ///FINO A DOVE: FIN DOVE CALCOLO LA DISTANZA, PIÙ CHE ALTRO FIN DOVE FACCIO TUTTO. ES: SE È 400, ALLORA
                        ///GUARDO (COSA GUARDO? quello che mi serve) fino a 400, appunto, pixel di distanza dal primo pixel
int distanza_isole=fino_a_dove-5; ///vedi sotto
int quanti_per_isola=3; ///QUANTI PER ISOLA: quanto è la distanza MASSIMA tra un pixel a una data distanza (distanza isole) di modo che si possano considerare
                        ///NON separati, cioè diciamo riunibili in uno stesso punto. Secondo la geometria, dovrebbe essere addirittura 1, non di più.

///VARIABILI PER LA FUNZIONE--------------
map<ii, int> distanze; ///la matrice dove il pixel in pos(x,y) è a distanza d dal primo nero trovato
int distanza_max=0;

//int vett_quanti[100000]; ///ma vett_quanti è inutilissimo!
vector<int>vettore_x[100000];
vector<int>vettore_y[100000];///100mila vuol dire che possiamo arrivare fino alla distanza di 100mila pixel (esagerato) (vedi NOTA)

//vector<int> isola_x;
//vector<int> isola_y; ///si salva il centro di ogni isola. la size è il numero di isole
vector<int> hdng_isole;
int linea_x[10000];
int linea_y[10000];

queue<Point> coda;

Mat colore, grigio;


int angolo_tra(float x1, float y1, float x2, float y2){

    swap(y1,y2);
    if(x1==x2)
        if(y1<y2) return 90;
        else      return 270;
    if(y1==y2)
        if(x1<x2) return 0;
        else      return 180;


    int angolo=(atan((y2-y1)/(x2-x1))*57.3);

    short seno;
    if(y2 > y1) seno=+1;
    else        seno=-1;

    if(angolo * seno > 0) return angolo;
    else                  return angolo+180;


}

float angolo_medio_linea(int* lineax, int* lineay, int lungh, int l, int step){

	//cout<<"INIZIO CALCOLO ANGOLO LINEA"<<endl;getchar();
	long long int media=0;
	int t=0, c=0;
for(int i=0; i<lungh-l; i+=step){

    c++;
    colore.at<Vec3b>(lineay[i+l],lineax[i+l]) = (Vec3b){0,0,255};
    //cout<<"finale : "<<lineax[i+l]<<" "<<lineay[i+l]<<"\n iniziale:"<<lineax[i]<<" "<<lineay[i]<<endl;
        t=angolo_tra(lineax[i],lineay[i],lineax[i+step],lineay[i+step]);
		media+=(int)t;
	//cout<<t<<" "<<media<<endl;getchar();
	colore.at<Vec3b>(lineay[i],lineax[i]) = (Vec3b){255,255,255};

    //imshow("colore",colore);
    //namedWindow("colore", WINDOW_AUTOSIZE);
    //waitKey(0);
	}
	//cout<<"campionis: "<<lungh<<" "<<step<<" "<<((int)lungh/step)<<endl;
	media=media/c;
    cout<<media<<endl;
    cout<<angolo_tra(lineax[0],lineay[0],lineax[lungh],lineay[lungh]);
	return media;
}

void bfs_principaple(Mat& grigio){ ///COLORA "colore" e riempie la matrice DISTANZE

int primox, primoy, lastx;
    ///NOTA PER TUTTO STO CESSO
    ///IL MAT::AT(A,B) FUNZIONA CHE A SONO LE RIGHE (QUINDI LA Y) E B LE COLONNE (LA X)
for(int i=grigio.rows-1; i>=0; i--)  ///i:righe
{
    for(int j=grigio.cols -1; j>=0; j--) ///j:colonne; CERCA IL PRIMO NERO A PARTIRE DA DESTRA
    {
        if ((int)grigio.at<uchar>(i,j) <=soglianero )
        {
            primox=j;
            primoy=i;
            j=-1;///esci dal for
        }
    }

    for(int j=0; j<grigio.cols; j++) ///QUANDO HA TROVATO IL PRIMO NERO A DESTRA, LO CERCA DA SINISTRA
    {
        if ((int)grigio.at<uchar>(i,j) <=soglianero )
        {
            lastx=j;

            //escimi il for
            j=grigio.cols+10;
            i=-1;  ///QUANDO HA TROVATO IL NERO DX E IL NERO SX CHE STANNO PIÙ IN BASSO DI TUTI,
                    ///esce da tutti i for possibili, ha trovato le cordinate esatte del primo nero
        }
    }

}
primox=(primox+lastx)/2;
colore.at<Vec3b>(primoy, primox)=(Vec3b){0,0,250};
//cout<<"HO TROVATO UN NERO!"<<primox<<" "<<primoy<<endl;

///--------------------<<BFS>>---------------///

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
            int distcorr= distanze[iimake(xcorrente,ycorrente)];
            if((xprossimo>=0 && xprossimo<grigio.cols) &&
                (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                &&
                (distanze.count(iimake(xprossimo,yprossimo)) == 0 )&& ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero) ///SE È UN NERO,
                )
            {

                coda.push(Point(xprossimo,yprossimo));    ///metti il prossimo nodo nella coda
                distanze[iimake(xprossimo,yprossimo)] =  distanze[iimake(xcorrente,ycorrente)]+1; ///setta la distanza del prossimo a la tua +1
                int distanza = distcorr+1; ///e salvala nella variabile "distanza"
                //vett_quanti[distanza]++; ///di che c'è un altro punto a quella distanza
                if(distanza> distanza_max)
                    distanza_max=distanza;
                calcola_colore(distanza);  ///colora il punto sulla immagine
                colore.at<Vec3b>(yprossimo,xprossimo) = (Vec3b){colorazione[0],colorazione[1],colorazione[2]};

                vettore_x[distanza].push_back(xprossimo);
                vettore_y[distanza].push_back(yprossimo);

             } //endif
        } //endfor


}//endwhile; fine bfs

}

void disegna_linea(){
    ///--<<CALCOLO DELLA MEDIA, PER VEDERE DOVE VA LA TRAIETTORIA DELLA LINEA>>----
int mediax;
int mediay;
for(int i=1; i<fino_a_dove; i++)
{
    mediax=0;
    mediay=0;
    //cout<<"I PIXELS A DISTANZA "<<i<<"SONO "<<vettore_x[i].size()<<" "<<vettore_y[i].size()<<" "<<vett_quanti[i]<<endl;//getchar(); //questi tre numeri dovrebbero SEMPRE essere uguali, o qualcosa non va
    for(int j=0; j<vettore_x[i].size(); j++)
    {
        //cout<<"       "<<vettore_x[i][j]<<"  "<<vettore_y[i][j]<<endl;
        mediax=mediax+vettore_x[i][j];
        mediay=mediay+vettore_y[i][j];
    }
    mediax=mediax /vettore_x[i].size();
    mediax=(int)mediax;
    mediay=mediay /vettore_y[i].size();
    mediay=(int)mediay;

    linea_x[i]=(int)mediax;
    linea_y[i]=(int)mediay;
    colore.at<Vec3b>(linea_y[i],linea_x[i])=(Vec3b){250,250,250};

    //cout<<"LA MEDIA ERA "<<mediax<<" "<<mediay<<endl<<endl;
}
}

int bfs_isole(Mat& grigio,int distanza_isole_funz){
    ///----<<LA CONTROBFS PER VEDERE QUALI SONO LE "ISOLE", cioè le ramificazioni----
map<ii, bool> usati;

int numero_isole=0;
for(int i=0; i<vettore_x[distanza_isole_funz].size(); i++)
    if(usati.count(iimake(vettore_x[distanza_isole_funz][i],vettore_y[distanza_isole_funz][i]))==0)
    {
        //cout<<"siamo !!!ALL INIZIO!! di un isola con";
        numero_isole++;
        usati[iimake(vettore_x[distanza_isole_funz][i],vettore_y[distanza_isole_funz][i])]=true;
        coda.push(Point(vettore_x[distanza_isole_funz][i],vettore_y[distanza_isole_funz][i]));

        //int mediax=0;
        //int mediay=0;
        //int pixel_per_isola=0;
        while(!coda.empty()) ///NOTA, ERA VERAMENTE UN CESSO DA IMPLEMENTARE IL FATTO DELLA DISTANZA MINIMA TRA I PIXEL ISOLA, COSÌ LASCIAI PERDERE
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
                    ii  proxnodo=iimake(xprossimo,yprossimo);
                    if((xprossimo>=0 && xprossimo<grigio.cols) && (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                        &&
                        (usati.count(proxnodo) == 0 )&&  (abs(distanze[proxnodo]-distanza_isole_funz) <=quanti_per_isola) && ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                        ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero) ///SE È UN NERO,
                        )

                    {
                        usati[proxnodo]=true;
                        //cout<<"assert "<<usati.count(proxnodo)<<" == "<<usati[iimake(xprossimo,yprossimo)]<<" =? 1"<<endl;
                        coda.push(Point(xprossimo,yprossimo));
                         colore.at<Vec3b>(yprossimo,xprossimo) = (Vec3b){250,250,0};

                         //mediax+=xprossimo;
                         //mediay+=yprossimo;
                         //pixel_per_isola++;
                    }
                }
        }//endwhile

        //mediax=mediax/pixel_per_isola;
        //mediay=mediay/pixel_per_isola;
        //cout<<"media calcolata no  vettori : "<<mediax<<" "<<mediay;

        //isola_x.push_back(mediax);
        //isola_y.push_back(mediay);
    }//endfor-if:trova ogni isola

    return numero_isole;
}//endfunction

void heading_isole(Mat& grigio){
    //cout<<"ciao";
    map<ii, bool> usati;
    Point corrente;
    int xprossimo,yprossimo;
    ii proxnodo;
    int mediax[500], mediay[500];
    int quantix[500], quantiy[500]; //non mettiamo i vector del cazzo, sono lenti

    int dist_proxnodo;

    for(int i=0; i<vettore_x[distanza_isole].size(); i++)
        if(usati.count(iimake(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i])) == 0)
        {
           for(int k=0; k<500; k++)
            {
                mediax[k]=0;
                mediay[k]=0;
                quantix[k]=0;
                quantiy[k]=0;
            }
            int max_dist_isola=0;

            coda.push(Point(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i]));
            while(!coda.empty()){
                //cout<<"siamo nella bfs  ";
                corrente=coda.front();
                coda.pop();
                //cout<<corrente.x<<" "<<corrente.y<<" "<<distanze[iimake(corrente.x,corrente.y)]<<endl;
                for(int k=-1; k<=1; k++)
                    for(int j=-1; j<=1; j++)   ///PER OGNI VICINO DEL PIXEL CORRENTE CONTROLLA SE LO DEVI PRENDERE
                    {
                        xprossimo = corrente.x+k;
                        yprossimo = corrente.y+j;
                        proxnodo=iimake(xprossimo,yprossimo);
                        dist_proxnodo=distanze[proxnodo];
                        if(dist_proxnodo > max_dist_isola)
                            max_dist_isola = dist_proxnodo;

                        if((xprossimo>=0 && xprossimo<grigio.cols) && (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                            &&
                            ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero)&& ///SE È UN NERO,
                            (dist_proxnodo >= distanza_isole) && (usati.count(proxnodo) == 0 )  ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                            )

                        {
                            usati[proxnodo]=true;
                            //cout<<"assert "<<usati.count(proxnodo)<<" == "<<usati[iimake(xprossimo,yprossimo)]<<" =? 1"<<endl;
                            coda.push(Point(xprossimo,yprossimo));
                             mediax[dist_proxnodo-distanza_isole]+=xprossimo;
                             mediay[dist_proxnodo-distanza_isole]+=yprossimo;
                             quantix[dist_proxnodo-distanza_isole]++;
                             quantiy[dist_proxnodo-distanza_isole]++;
                        }
                    }
            ///
            }//endwhile
        for(int k=0; k<500 && quantix[k]>=1; k++)
        {
            //cout<<"sono a dist "<<k+distanza_isole<<" e ho "<<mediax[k-distanza_isole]<<" somma x per "<<quantix[k-distanza_isole]<<endl;
            mediax[k]=(int)(mediax[k]/quantix[k]);
            mediay[k]=(int)(mediay[k]/quantiy[k]);
        }

        hdng_isole.push_back(angolo_medio_linea(mediax,mediay,max_dist_isola-15-distanza_isole, 10,10));
        cout<<"direzione dell isola"<<hdng_isole.size()<<" "<<hdng_isole[hdng_isole.size()-1]<<endl;//getchar();

        }//endif/for
return ;
}

int main(int argc, char** argv )
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    /*namedWindow("Line",1);
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
        for(int contrl=0;contrl<5;contrl++){
        cap >> frame; // get a new frame from camera
        lns=Mat::zeros( 240, 320, CV_8UC3 );

        }
        /*frame=imread(argv[1], CV_LOAD_IMAGE_COLOR);
        if ( !frame.data )
            {
                printf("No image data \n");
                return -1;
            }
        cout<<"letto\n";*/
        imshow("Line", frame);
        cvtColor(frame, edges, CV_BGR2GRAY);
        //imshow("Line", edges);
        threshold(edges,thr, 70, 255, THRESH_BINARY);

        grigio=thr;
        cvtColor(grigio,colore, CV_GRAY2BGR);

        //Erosion(0,0);
        ///Trovalinea

        for(int i=0; i<10000; i++)
        {
        linea_x[i]=0;
        linea_y[i]=0;
        //vett_quanti[i]=0;
        }
        cvtColor(grigio,colore, CV_GRAY2BGR);
        distanze.clear();
        ///FINZE INIZIALIZZAZIONE---------------

        bfs_principaple(grigio);
        fino_a_dove=distanza_max;
        cout<<"dist max :"<<distanza_max<<endl;

        /*al massimo*/
        for(int dove=2; dove<fino_a_dove; dove+=50){
            if(bfs_isole(grigio,dove)>=2){
                fino_a_dove=dove+50;
                break;
             }
        }

        int ub=fino_a_dove;
        int lb=2;
        int mid=(ub+lb)/2;

        while(ub>lb+1){
            mid=(ub+lb)/2;
            if(bfs_isole(grigio,mid)==1)
                lb=mid;
            else
                ub=mid;
        }
            distanza_isole=ub+5;
            fino_a_dove=distanza_isole-15;

        cout<<"una fatta\n"<<"DISTANZA ISOLE : "<<distanza_isole;
        disegna_linea();
        cout<<"due\n";
        int direzione_principale = angolo_medio_linea(linea_x,linea_y,fino_a_dove, 10,3);
        cout<<"L'ANGOLO ERA CIRCA "<<direzione_principale<<endl;

        cout<<" a "<<distanza_isole<<" ci sono n isole: "<<bfs_isole(grigio,distanza_isole)<<endl;
        //getchar();
        heading_isole(grigio);

        cout<<"\nfine--------------------------------------------\n";

        namedWindow("finito", WINDOW_FULLSCREEN);
        imshow("finito",colore);
         imwrite("immagine2.png", colore);


        Dilation(0,0);
        imshow("Original",frame);
        //imshow("Line", thr);
        //imshow("Elab", lns);
        //if(waitKey(30) >= 0) break;
        //waitKey(0);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
