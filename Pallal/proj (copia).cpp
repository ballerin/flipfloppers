#include <stdio.h>
#include<cstdlib>
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include<vector>
#include<queue>
using namespace cv;
using namespace std;

struct Mypoint{
int x;
int y;
int dist;

};


int colorazione[3]={0,0,0};
int fino_a_dove=100;

void calcola_colore(int dist)
{

int qualecolore = (dist/250)%3;
int intensita=(dist%250);

for(int i=0; i<3; i++)
    colorazione[i]=0;

colorazione[qualecolore]=intensita;

}

/**
NOTE DI VERSIONE(QUANTO FA FIGO DIRLO):

QUESTA È UNA COPIA CHE ERA FUNZIONANTE AL 1 SETT 2015, DOVE FA LA BFS E TUTTO MA LA MATRICE DELLE DISTANZE NON È
- NE ALLOCCATA SULLO SWAP
- NE IMPLEMENTATA CON UN MAP
(COSA CHE VORRÒ POI FARE)

È UNA SEMPLICE MATRICE DI INT CHE SULLO STACK PIÙ GRANDE DI 500X500 DA ERRORE!

*/

int main(int argc, char** argv )
{
cout<<"ciao"<<endl;
Mat grigio=imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
if ( !grigio.data )
    {
        printf("No image data \n");
        return -1;
    }
Mat colore;
cout<<"debug 0"<<grigio.rows<<"  "<<grigio.cols<<endl;
int distanze[grigio.cols][grigio.rows];
cout<<"debug 1"<<endl;
int vett_mediax[100000];
int vett_mediay[100000];
int vett_quanti[100000];
vector<int>vettore_x[100000];
vector<int>vettore_y[100000];

for(int i=0; i<10000; i++)
{
vett_mediax[i]=0;
vett_mediay[i]=0;
vett_quanti[i]=0;
}
cout<<"ho finito l'init"<<endl;
for(int i=0; i<grigio.cols; i++)
    for(int j=0; j<grigio.rows; j++)
    {
        distanze[i][j] = -1;
    }
cout<<"le distanze 00"<<distanze[1][198]<<endl;

cvtColor(grigio,colore, CV_GRAY2BGR);

///PARTIAMO DAL BASSO A INDIVIDUARE IL PRIMO NEGRO!

int primox, primoy, lastx;

for(int i=grigio.rows-1; i>=0; i--)
{
    for(int j=grigio.cols -1; j>=0; j--)
    {
        if ((int)grigio.at<uchar>(i,j) <=20 )
        {
            primox=j;
            primoy=i;

            //escimi il for
            j=-1;
        }
    }
    ///QUANDO HA TROVATO IL NERO A DESTRA, NE TROVA ANCHE L'ESTREMO A SINISTRA
    for(int j=0; j<grigio.cols; j++)
    {
        if ((int)grigio.at<uchar>(i,j) <=20 )
        {
            lastx=j;

            //escimi il for
            j=grigio.cols+10;
            i=-1;
        }
    }

}
primox=(primox+lastx)/2;

///NOTA PER TUTTO STO CESSO
///IL MAT::AT(A,B) FUNZIONA CHE A SONO LE RIGHE (QUINDI LA Y) E B LE COLONNE (LA X)

{///DENTRO STE PARENTESI PER COMODITA' XEKE IN CODEBLOCS SI PUÒ CHIUDERE
colore.at<Vec3b>(primoy, primox)=(Vec3b){0,0,250};
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
}
cout<<"HO TROVATO UN NERO!"<<primox<<" "<<primoy<<endl;
//cout<<grigio<<endl;
queue<Point> coda;


coda.push(Point(primox, primoy));
distanze[primox][primoy] =0;
cout<<"RIGHE E COLONNE DELLA MATRICE SU CUI OPERO"<<grigio.rows<<"  "<<grigio.cols<<endl;getchar();
while(!coda.empty())
{
    Point corrente = coda.front();
    coda.pop();

    int xcorrente=corrente.x;
    int ycorrente=corrente.y;
    //cout<<"sono a "<<corrente.x<<" "<<corrente.y<<endl;//getchar();;
    //cout<<"le prossime coordinate saranno ";
    for(int i=-1; i<=1; i++)
        for(int j=-1; j<=1; j++)
        {
            int xprossimo = xcorrente+i;
            int yprossimo = ycorrente+j;
            //cout<<"prima la x e poi la y "<<xprossimo<<" "<<yprossimo<<"e la sua distanza era "<<distanze[xprossimo][yprossimo]<<endl;
            if((xprossimo>=0 && xprossimo<grigio.cols) && (yprossimo>=0 && yprossimo<grigio.rows))
            {
                    if( (distanze[xprossimo][yprossimo] == -1 )&& ( (int)grigio.at<uchar>(yprossimo,xprossimo) <= 20)  )
                    {
                        //cout<<"e le prendo"<<endl;//getchar();;
                        coda.push(Point(xprossimo,yprossimo));
                        distanze[xprossimo][yprossimo] = distanze[xcorrente][ycorrente]+1;
                        int distanza = distanze[xprossimo][yprossimo];
                        calcola_colore(distanza);
                        colore.at<Vec3b>(yprossimo,xprossimo) = (Vec3b){colorazione[0],colorazione[1],colorazione[2]};
                        cout<<"la distanza era "<<distanza<<endl;

                        int divisore = vett_quanti[distanza]; ///METTERE LA REFERENCE. CHE FIGATA SAREBBE?
                        ///MEDIA DINAMICA
                        vett_mediax[distanza]= ( (divisore*vett_mediax[distanza]) + (xprossimo) )/(divisore+1);
                        vett_mediay[distanza]= ( (divisore*vett_mediay[distanza]) + (yprossimo) )/(divisore+1);

                        //reference
                        vett_quanti[distanza]++;

                        ///media statica
                        vettore_x[distanza].push_back(xprossimo);
                        vettore_y[distanza].push_back(yprossimo);

                    }
             }
        }


}
cout<<"FINE BFS!!"<<endl;

if(argc>=3)
{
     stringstream s;
        s<<argv[2];
        s>>fino_a_dove;
}



float mediax;
float mediay;
for(int i=1; i<fino_a_dove; i++)
{
    mediax =0.0;
    mediay =0.0;
    cout<<"I PIXELS A DISTANZA "<<i<<"SONO "<<vettore_x[i].size()<<" "<<vettore_y[i].size()<<" "<<vett_quanti[i]<<endl;//getchar();
    for(int j=0; j<vettore_x[i].size(); j++)
    {
        cout<<"       "<<vettore_x[i][j]<<"  "<<vettore_y[i][j]<<endl;
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

    cout<<"LA MEDIA ERA "<<mediax<<" "<<mediay<<endl<<endl;



}



namedWindow("COLORE", WINDOW_NORMAL);
imshow("COLORE",colore);
 imwrite("immagine2.png", colore);

namedWindow("grigio", WINDOW_NORMAL);
imshow("grigio",grigio);
  waitKey(0);
    return 0;
}


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
            if( (distanze[corrente.x + i][corrente.y + j] == -1 )&& ( (int)grigio.at<uchar>(corrente.x+i, corrente.y+j) <= 20)  )
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
