#include <stdio.h>
#include<cstdlib>
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include<vector>
#include<queue>
#include<map>
#include<math.h>

#define ii pair<int,int>

using namespace cv;
using namespace std;

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

Mat colore;

/**
CONSIDERANDO CHE VETTORE_X (e y)  lo si usa nella bfs per le isole, alla fine, secondo me, non è nemmeno così malvagia l'idea di usarlo
per calcolare la media e poi la linea bianca A POSTERIORI, invece che calcolarla dinamicamente duarante la bfs. Così facendo si risparmia addirittura un vettore
*/

int angolo_tra(int x1, int y1, int x2, int y2){

    swap(y1,y2);
    if(x1==x2)
        if(y1<y2) return 90;
        else      return 270;
    if(y1==y2)
        if(x1<x2) return 0;
        else      return 180;


    int angolo=atan((y2-y1)/(x2-x1))*57.3;

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
	//cout<<t<<" "<<media<<endl;//getchar();
	colore.at<Vec3b>(lineay[i],lineax[i]) = (Vec3b){0,0,255};

    imshow("colore",colore);
    namedWindow("colore", WINDOW_FULLSCREEN);
    //waitKey(0);
	}
	//cout<<"campionis: "<<lungh<<" "<<step<<" "<<((int)lungh/step)<<endl;
	media=media/c;
    cout<<media<<endl;
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
    cout<<"ciao";
    map<ii, bool> usati;
    Point corrente;
    int xprossimo,yprossimo;
    ii proxnodo;
    int mediax[500], mediay[500];
    int quantix[500], quantiy[500]; //non mettiamo i vector del cazzo, sono lenti

    int dist_proxnodo;

    for(int i=0; i<vettore_x[distanza_isole].size(); i++){
        cout<<"siamo a "<<i<<"di "<<vettore_x[distanza_isole].size()<<endl;
        if(usati.count(iimake(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i])) == 0)
        {

           cout<<"isola!";
           for(int k=0; k<500; k++)
            {
                mediax[k]=0;
                mediay[k]=0;
                quantix[k]=0;
                quantiy[k]=0;
            }

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

            }//endwhile
        for(int k=0; k<500 && quantix[k]>=1; k++)
        {
            //cout<<"sono a dist "<<k+distanza_isole<<" e ho "<<mediax[k-distanza_isole]<<" somma x per "<<quantix[k-distanza_isole]<<endl;
            mediax[k]=(int)(mediax[k]/quantix[k]);
            mediay[k]=(int)(mediay[k]/quantiy[k]);
        }

        hdng_isole.push_back(angolo_medio_linea(mediax,mediay,100, 10,10));
        cout<<"heding dell isola"<<hdng_isole.size()<<" "<<hdng_isole[hdng_isole.size()-1]<<endl;//getchar();

        }//endif/for
}//end for
cout<<"finito\n";
return;
}

int main(int argc, char** argv )
{

cout<<"ciao"<<endl;
Mat grigio=imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
if ( !grigio.data )
    {
        printf("No image data \n");
        return -1;
    }

///INIZIALIZZAZIONE-----------------
if(argc>=3)
{
     stringstream s;
        s<<argv[2];
        s>>fino_a_dove;
}
if(argc>=4)
{
     stringstream s;
        s<<argv[3];
        s>>distanza_isole;
}
if(argc>=5)
{
     stringstream s;
        s<<argv[4];
        s>>quanti_per_isola;
}


for(int i=0; i<10000; i++)
{
linea_x[i]=0;
linea_y[i]=0;
//vett_quanti[i]=0;
}
cvtColor(grigio,colore, CV_GRAY2BGR);
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

cout<<"una fatta\n";
disegna_linea();
cout<<"due\n";
int direzione_principale = angolo_medio_linea(linea_x,linea_y,fino_a_dove, 10,3);
cout<<"L'ANGOLO ERA CIRCA "<<direzione_principale<<endl;

cout<<" a "<<distanza_isole<<" "<<bfs_isole(grigio,distanza_isole)<<endl;
//getchar();
heading_isole(grigio);

cout<<"\nfine--------------------------------------------\n";

namedWindow("finito", WINDOW_FULLSCREEN);
imshow("finito",colore);
 imwrite("immagine2.png", colore);
/*
namedWindow("grigio", WINDOW_NORMAL);
imshow("grigio",grigio);*/
  waitKey(0);
    return 0;
}


/*colore.at<Vec3b>(primoy, primox-1)=(Vec3b){0,0,250};
colore.at<Vec3b>(primoy, primox+1)=(Vec3b){0,0,250};
colore.at<Vec3b>(primoy, primox-2)=(Vec3b){0,0,250};
colore.at<Vec3b>(primoy, primox+2)=(Vec3b){0,0,250};*/

/*colore.at<Vec3b>(primox, primoy-1)=(Vec3b){0,0,250};
colore.at<Vec3b>(primox, primoy+1)=(Vec3b){0,0,250};
colore.at<Vec3b>(primox, primoy-2)=(Vec3b){0,0,250};
colore.at<Vec3b>(primox, primoy+2)=(Vec3b){0,0,250};
colore.at<Vec3b>(primox-1, primoy)=(Vec3b){0,0,250};
colore.at<Vec3b>(primox+1, primoy)=(Vec3b){0,0,250};
colore.at<Vec3b>(primox-2, primoy)=(Vec3b){0,0,250};
colore.at<Vec3b>(primox+2, primoy)=(Vec3b){0,0,250};*/


