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
int soglianero=20;
int fino_a_dove=100;   ///FINO A DOVE: FIN DOVE CALCOLO LA DISTANZA, PIÙ CHE ALTRO FIN DOVE FACCIO TUTTO. ES: SE È 400, ALLORA
                        ///GUARDO (COSA GUARDO? quello che mi serve) fino a 400, appunto, pixel di distanza dal primo pixel
int distanza_isole=fino_a_dove-5; ///vedi sotto
int quanti_per_isola=3; ///QUANTI PER ISOLA: quanto è la distanza MASSIMA tra un pixel a una data distanza (distanza isole) di modo che si possano considerare
                        ///NON separati, cioè diciamo riunibili in uno stesso punto. Secondo la geometria, dovrebbe essere addirittura 1, non di più.

///VARIABILI PER LA FUNZIONE--------------
map<ii, int> distanze; ///la matrice dove il pixel in pos(x,y) è a distanza d dal primo nero trovato
map<ii, bool> usati;  ///serve nella "contro"bfs, per vedere se ho già usato un punto

int vett_quanti[100000]; ///ma vett_quanti è inutilissimo!
vector<int>vettore_x[100000];
vector<int>vettore_y[100000];///100mila vuol dire che possiamo arrivare fino alla distanza di 100mila pixel (esagerato)
Mat colore;


float angolo_medio_linea(int* mediax, int* mediay, int lungh, int l, int step){
	
	cout<<"INIZIO CALCOLO ANGOLO LINEA"<<endl;getchar();
	long long int media=0.0;
	float t=0.0;
for(int i=0; i<lungh-l; i+=step){
	
  
      colore.at<Vec3b>(mediay[i+l],mediax[i+l]) = (Vec3b){0,0,255};
	cout<<"finale : "<<mediax[i+l]<<" "<<mediay[i+l]<<"\n iniziale:"<<mediax[i]<<" "<<mediay[i]<<endl;
        if((mediax[i+l]-mediax[i])!=0)
            t=atan((mediay[i+l]-mediay[i]) / (mediax[i+l]-mediax[i]) );
        else
            t=100;
		media+=(int)t;
	cout<<t<<endl;//getchar();
	namedWindow("COLORE", WINDOW_FULLSCREEN);
imshow("COLORE",colore);
waitKey(0);
	}
	media=media/(lungh/l);

	return media;
}

int bfs_principaple(){
  
}

int main(int argc, char** argv )
{

int vett_mediax[100000];
int vett_mediay[100000];    

cout<<"ciao"<<endl;
Mat grigio=imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
if ( !grigio.data )
    {
        printf("No image data \n");
        return -1;
    }



queue<Point> coda;

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
vett_mediax[i]=0;
vett_mediay[i]=0;
vett_quanti[i]=0;
}
cvtColor(grigio,colore, CV_GRAY2BGR);
///FINZE INIZIALIZZAZIONE---------------

///PARTIAMO DAL BASSO A INDIVIDUARE IL PRIMO NERO!

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
cout<<"HO TROVATO UN NERO!"<<primox<<" "<<primoy<<endl;

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

            if((xprossimo>=0 && xprossimo<grigio.cols) &&
                (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                &&
                (distanze.count(iimake(xprossimo,yprossimo)) == 0 )&& ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero) ///SE È UN NERO,
                )
            {

                coda.push(Point(xprossimo,yprossimo));    ///metti il prossimo nodo nella coda
                distanze[iimake(xprossimo,yprossimo)] = distanze[iimake(xcorrente,ycorrente)]+1; ///setta la distanza del prossimo a la tua +1
                int distanza = distanze[iimake(xprossimo,yprossimo)]; ///e salvala nella variabile "distanza"
                vett_quanti[distanza]++; ///di che c'è un altro punto a quella distanza

                calcola_colore(distanza);  ///colora il punto sulla immagine
                colore.at<Vec3b>(yprossimo,xprossimo) = (Vec3b){colorazione[0],colorazione[1],colorazione[2]};

                vettore_x[distanza].push_back(xprossimo);
                vettore_y[distanza].push_back(yprossimo);

             } //endif
        } //endfor


}//endwhile; fine bfs

///--<<CALCOLO DELLA MEDIA, PER VEDERE DOVE VA LA TRAIETTORIA DELLA LINEA>>----
float mediax;
float mediay;
for(int i=1; i<fino_a_dove; i++)
{
    mediax=0.0;
    mediay=0.0;
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

    vett_mediax[i]=(int)mediax;
    vett_mediay[i]=(int)mediay;
    colore.at<Vec3b>(vett_mediay[i],vett_mediax[i])=(Vec3b){250,250,250};

    //cout<<"LA MEDIA ERA "<<mediax<<" "<<mediay<<endl<<endl;
}

float direzione_principale = angolo_medio_linea(vett_mediax, vett_mediay,fino_a_dove, 10,3);
cout<<"L'ANGOLO ERA CIRCA "<<direzione_principale<<endl;

///----<<LA CONTROBFS PER VEDERE QUALI SONO LE "ISOLE", cioè le ramificazioni----
for(int i=0; i<vettore_x[distanza_isole].size(); i++)
{
    //la coda dovrebbe essere vuota
    if(usati.count(iimake(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i]))==0)
    {
        cout<<"siamo !!!ALL INIZIO!! di un isola con";
        usati[iimake(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i])]=true;
        coda.push(Point(vettore_x[distanza_isole][i],vettore_y[distanza_isole][i]));
        //cout<<" "<< vettore_x[distanza_isole][i]<<"  "<<vettore_y[distanza_isole][i]<<endl;getchar();

        while(!coda.empty()) ///NOTA, ERA VERAMENTE UN CESSO DA IMPLEMENTARE IL FATTO DELLA DISTANZA MINIMA TRA I PIXEL ISOLA, COSÌ LASCIAI PERDERE
        {

            Point corrente = coda.front();
            coda.pop();
            int xcorrente=corrente.x;
            int ycorrente=corrente.y;
            //cout<<"punto della bfs:"<<xcorrente<<" "<<ycorrente<<" a distanza "<<distanze[iimake(xcorrente,ycorrente)];//getchar();

            for(int i=-1; i<=1; i++)
                for(int j=-1; j<=1; j++)   ///PER OGNI VICINO DEL PIXEL CORRENTE CONTROLLA SE LO DEVI PRENDERE
                {
                    int xprossimo = xcorrente+i;
                    int yprossimo = ycorrente+j;
                    ii  proxnodo=iimake(xprossimo,yprossimo);
                    if((xprossimo>=0 && xprossimo<grigio.cols) && (yprossimo>=0 && yprossimo<grigio.rows)  ///SE È DENTRO I CONFINI DELLA MATRICE,
                        &&
                        (usati.count(proxnodo) == 0 )&&  (abs(distanze[proxnodo]-distanza_isole) <=quanti_per_isola) && ///BFS:SE NON LO HAI GIA PRESO IN CONSIDERAZIONE,
                        ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= soglianero) ///SE È UN NERO,
                        )

                    {
                        usati[proxnodo]=true;
                        //cout<<"assert "<<usati.count(proxnodo)<<" == "<<usati[iimake(xprossimo,yprossimo)]<<" =? 1"<<endl;
                        coda.push(Point(xprossimo,yprossimo));
                         colore.at<Vec3b>(yprossimo,xprossimo) = (Vec3b){250,250,0};
                    }
                }
        }
    cout<<"ho scoperto un isola, e la ho colorata\n";//getchar();

    ///QUI SI DEVONO POI METTERE PUTTANATE TIPO LA MEDIA ECCETERA ECCETERA ECCETERA :)
    }


}
namedWindow("COLORE", WINDOW_NORMAL);
imshow("COLORE",colore);
 imwrite("immagine2.png", colore);

namedWindow("grigio", WINDOW_NORMAL);
imshow("grigio",grigio);
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

/*

COPIA DI BACKUP DELLA BFS

coda.push(Point(primox, primoy));
distanze[primoy][primox] =0;
cout<<"dio merda";
while(!coda.empty())
{
    Point corrente = coda.front();
    coda.pop();

    int xcorrente=corrente.x;
    int ycorrente=corrente.y;
 //   cout<<"sono a "<<corrente.x<<" "<<corrente.y<<endl;getchar();
    for(int i=-1; i<=1; i++)
        for(int j=-1; j<=1; j++)
        {
            if( (distanze[corrente.x + i][corrente.y + j] == -1 )&& ( (int)grigio.at<uchar>(corrente.x+i, corrente.y+j) <= soglianero)  )
                {
                    coda.push(Point(corrente.x + i,corrente.y + j));
                    distanze[corrente.x + i][corrente.y + j] = distanze[corrente.x][corrente.y]+1;
                    calcola_colore();
                    colore.at<Vec3b>(corrente.x + i,corrente.y + j) = (Vec3b){colorazione[0],colorazione[1],colorazione[2]};

                    int distanza = distanze[corrente.x + i][corrente.y + j];
                    int divisore = vett_quanti[distanza]; ///METTERE LA REFERENCE. CHE FIGATA SAREBBE?

                    vett_mediax[distanza]= ( (divisore*vett_mediax[distanza]) + (corrente.x+i) )/(divisore+1);
                    vett_mediay[distanza]= ( (divisore*vett_mediay[distanza]) + (corrente.y+j) )/(divisore+1);

                    //reference
                    vett_quanti[distanza]++;

                }
        }
}

*/
