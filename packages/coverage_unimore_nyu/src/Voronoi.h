#ifndef VORONOI_H_INCLUDED
#define VORONOI_H_INCLUDED

#pragma once
// My includes
#include "FortuneAlgorithm.h"
#include <iostream>

#define M_PI   3.14159265358979323846  /*pi*/

const int DEBUG = 0;

//***********************************************************************************************************************************
//-------------------------------- FUNZIONI AUSILIARIE INIZIALI (trasformazione coordinate + sensing) ---------------------------------
//Funzione che esclude dal calcolo i punti esterni ad AreaBox
template<typename T>
std::vector<Vector2<T>> adjustPointsVector(const std::vector<Vector2<T>>& points, Box<T> AreaBox)
{
    auto adj_points = std::vector<Vector2<T>>();

    for(const auto& point : points){
        //Esclusione punti esterni
        if(AreaBox.contains(point)){
            adj_points.push_back(point);
        }
    }

    return adj_points;
}

//Funzione che rielabora il vettore input dei punti globali in coordinate locali per il singolo punto mainPoint
template<typename T>
std::vector<Vector2<T>> reworkPointsVector(const std::vector<Vector2<T>>& points, Vector2<T> mainPoint)
{
    auto rwrk_points = std::vector<Vector2<T>>();

    //Trasformazione di coordinate (esterno al costruttore del diagramma)
    for(const auto& point : points){
        auto rwrk_point = Vector2<T>{point.x-mainPoint.x,point.y-mainPoint.y};
        rwrk_points.push_back(rwrk_point);
    }

    //stampa di debug per checkare se la funzione fa il suo compito
    if (DEBUG >= 1)
    {
        std::cout << "\nCoordinate locali punti: " << std::endl;
        for(std::size_t i=0; i<rwrk_points.size(); ++i){
            std::cout << rwrk_points[i] << std::endl;
        }
    }

    return rwrk_points;
}

// Funzione per descrivere gli ostacoli in coordinate locali rispetto a posizione robot
template<typename T>
Box<T> reworkObstacle(Box<T> ObstacleBox, Vector2<T> global_pos)
{
    Box<T> adjObstacleBox;

    // Get vertex coordinates
    Vector2<T> bl = {ObstacleBox.left, ObstacleBox.bottom};         // bottom-left point
    Vector2<T> tr = {ObstacleBox.right, ObstacleBox.top};         // top-right point

    // Define vector of vertexes
    std::vector<Vector2<T>> Vert = {bl,tr};

    // Rework points
    std::vector<Vector2<T>> adjVert = reworkPointsVector(Vert, global_pos);

    adjObstacleBox.left = adjVert[0].x;
    adjObstacleBox.bottom = adjVert[0].y;
    adjObstacleBox.right = adjVert[1].x;
    adjObstacleBox.top = adjVert[1].y;

    return adjObstacleBox;
}

//Funzione filtraggio/esclusione punti esterni alla RangeBox (simula azione sensore del robot)
template<typename T>
std::vector<Vector2<T>> filterPointsVector(const std::vector<Vector2<T>>& rwrk_points, Box<T> RangeBox)
{
    auto flt_points = std::vector<Vector2<T>>();

    //Filtraggio
    for(std::size_t i=0; i<rwrk_points.size(); ++i){
        auto flt_point = rwrk_points[i];
        //Mantiene solo i punti vicini
        if((RangeBox.contains(rwrk_points[i])) && (rwrk_points[i]!=Vector2<T>{0.0,0.0})){
            flt_points.push_back(flt_point);
        }
    }

    //stampa di debug per checkare se la funzione fa il suo compito
    if (DEBUG >= 1)
    {
        std::cout << "\nPunti vicini rilevati: " << std::endl;
        for(std::size_t i=0; i<flt_points.size(); ++i){
            std::cout << "Punto " << i << flt_points[i] << std::endl;
        }
    }

    return flt_points;
}

//Funzione filtraggio/esclusione punti esterni alla RangeBox (simula azione sensore del robot) 
//Return also the ids fo the neighbors
template<typename T>
std::pair<std::vector<Vector2<T>>, std::vector<int>> filterPointsVectorNeighbor(const std::vector<Vector2<T>>& rwrk_points, Box<T> RangeBox)
{
    auto flt_points = std::vector<Vector2<T>>();
    std::vector<int> neighbors = std::vector<int>();

    //Filtraggio
    for(std::size_t i=0; i<rwrk_points.size(); ++i){
        auto flt_point = rwrk_points[i];
        //Mantiene solo i punti vicini
        if((RangeBox.contains(rwrk_points[i])) && (rwrk_points[i]!=Vector2<T>{0.0,0.0})){
            flt_points.push_back(flt_point);
            neighbors.push_back(i);
        }
    }

    //stampa di debug per checkare se la funzione fa il suo compito
    if (DEBUG >= 1)
    {
        std::cout << "\nPunti vicini rilevati: " << std::endl;
        for(std::size_t i=0; i<flt_points.size(); ++i){
            std::cout << "Punto " << i << flt_points[i] << std::endl;
        }
    }

    return std::make_pair(flt_points, neighbors);
}

template<typename T>
Box<T> adjustRangeBox(Box<T> RangeBox, Vector2<T> point, const double ROBOT_RANGE, Box<T> AreaBox)
{
    auto AdjRangeBox = RangeBox;    //default

    //Adattamento ai limiti del campo (AreaBox)
    if(point.x - AreaBox.left < ROBOT_RANGE){
        AdjRangeBox.left = AreaBox.left - point.x;
    }
    if(point.y - AreaBox.bottom < ROBOT_RANGE){
        AdjRangeBox.bottom = AreaBox.bottom - point.y;
    }
    if(AreaBox.right - point.x < ROBOT_RANGE){
        AdjRangeBox.right = AreaBox.right - point.x;
    }
    if(AreaBox.top - point.y < ROBOT_RANGE){
        AdjRangeBox.top = AreaBox.top - point.y;
    }

    return AdjRangeBox;
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//--------------------------------------------------- Diagramma Centralizzato ------------------------------------------------------
template<typename T>
Diagram<T> generateCentralizedDiagram(const std::vector<Vector2<T>>& points, Box<T> AreaBox)
{
    // Construct Diagram
    FortuneAlgorithm<T> algorithm(points, AreaBox);
    algorithm.construct();

    // Bound the diagram (Take the bounding box slightly bigger than the intersection box)
    algorithm.bound(Box<T>{-0.05 + AreaBox.left, -0.05 + AreaBox.bottom, 0.05 + AreaBox.right, 0.05 + AreaBox.top});
    Diagram<T> diagram = algorithm.getDiagram();

    // Intersect the diagram with a box
    if(diagram.getNbSites()!=1){
        diagram.intersect(AreaBox);
    }
    else{
        diagram.intersect_null(AreaBox);
    }

    return diagram;
}

//--------------------------------------------------- DIAGRAMMA DECENTRALIZZATO ----------------------------------------------------------------------------------------------------------------------
//Alla seguente funzione arrivano in input i punti gi� in coordinate locali e filtrati (quindi nel main dovr� essere preceduta dalle funzioni adjustRangeBox e reworkPointsVector(points, 0) mettendo come elemento 0 il punto di cui si vuole realizzare il diagramma decentralizzato
template<typename T>    //flt_points: punti gi� filtrati, RangeBox: per comodit�, point_global: informazione su posizione iniziale globale, ROBOT_RANGE e AREA_SIZE: informazioni utili per singolo robot
Diagram<T> generateDecentralizedDiagram(const std::vector<Vector2<T>>& flt_points, Box<T> RangeBox, const Vector2<T>& point_global, const double ROBOT_RANGE, Box<T> AreaBox)    //point_global corrisponde alle coordinate globali del punto centrale del diagramma rispetto all'area di lavoro (informazione conosciuta dal robot?)
{
    auto half_ROBOT_RANGE = ROBOT_RANGE/2;
    Box<double> HalfRangeBox{-half_ROBOT_RANGE, -half_ROBOT_RANGE, half_ROBOT_RANGE, half_ROBOT_RANGE};
    // Construct Diagram
    FortuneAlgorithm<T> algorithm(flt_points, HalfRangeBox, point_global);
    algorithm.construct();

    // Bound the diagram
    algorithm.bound(Box<T>{-half_ROBOT_RANGE-0.05, -half_ROBOT_RANGE-0.05, half_ROBOT_RANGE+0.05, half_ROBOT_RANGE+0.05});
    Diagram<T> diagram = algorithm.getDiagram();

    //adjustRangeBox andrebbe qui secondo me--> perch� il robot vede in base a RangeBox e aggiusta in AdjRangeBox in base alla conoscenza dei limiti del campo di lavoro e della sua posizione globale
    auto AdjRangeBox = adjustRangeBox(HalfRangeBox, point_global, half_ROBOT_RANGE, AreaBox);

    // Intersect the diagram with a box (diversificato in base al fatto che il diagramma abbia o meno altri siti oltre a quello centrale --> soluzione anti-crash)
    if(diagram.getNbSites()!=1){        //funzione di intersezione classica/originale nel caso di un numero di siti maggiore di 1
        diagram.intersect(AdjRangeBox);
    }
    else{                               //funzione anti-crash nel caso di diagramma con 1 solo sito, il diagramma coincide con la box esterna
        diagram.intersect_null(AdjRangeBox);    //OPPURE solo in questo caso RangeBox (usare Test Centroidi Decentralizzati nel main per far capire)
    }
    //Debug
    if (DEBUG >= 1)
    {
        std::cout << "Numero di siti: " << diagram.getNbSites() << std::endl;
        std::cout << "Numero di facce: " << diagram.getFaces().size() << std::endl;
        std::cout << "Numero di halfedge: " << diagram.getHalfEdges().size() << std::endl;
        std::cout << "Numero di vertici: " << diagram.getVertices().size() << std::endl;
    }

    return diagram;
}

template<typename T>
std::vector<Diagram<T>> generateDecentralizedDiagrams(const std::vector<Vector2<T>>& points, Box<T> RangeBox, const double ROBOT_RANGE, Box<T> AreaBox)
{
    //Vettore dei diagrammi di Voronoi
    auto diagrams = std::vector<Diagram<T>>();

    //Generatore iterativo sull'index
    for(std::size_t i=0; i<points.size(); ++i){
        if (DEBUG >= 1)
        {
            std::cout << "-------------------------------------\n";
            std::cout << "Diagramma " << i << " - " << "di " << points[i] << std::endl;
        }

        //Aggiustamento RangeBox in base ai limiti del campo --> fuori non dovrebbe rilevare punti
        //auto AdjRangeBox = adjustRangeBox(RangeBox, points[i], ROBOT_RANGE, AREA_SIZE);

        //Rielaborazione vettore "points" globale in coordinate locali
        auto rwrk_points = reworkPointsVector(points, points[i]);

        //Filtraggio siti esterni alla box (simula azione del sensore)
        auto flt_points = filterPointsVector(rwrk_points, /*Adj*/RangeBox);

        //Generazione Diagramma Decentralizzato
        auto diagram = generateDecentralizedDiagram(flt_points, /*Adj*/RangeBox, points[i], ROBOT_RANGE, AreaBox);
        diagrams.push_back(std::move(diagram));
    }

    return diagrams;
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//----------------------------- Compute centroids - Uniform Distribution - Green's Theorem - Gauss Theorem --------------------------

//Funzione calcolo vettore dei centroidi dove ogni elemento del vettore � il centroide del site0 di ogni diagramma del vettore dei diagrammi
//Centroidi geometrici (densit� uniforme (fi=1)) --> una volta debuggato nel complesso spostare in Diagram.h per lavorare sul singolo sito
template<typename T>
std::vector<Vector2<T>> computeCentroids(const std::vector<Diagram<T>>& diagrams)
{
    auto centroids = std::vector<Vector2<T>>();
    
    for(const auto& diagram : diagrams){
        centroids.push_back(computeCentroid(diagram));
    }
    return centroids;
}

//Calcolo Centroide Geometrico del poligono definito dal diagramma voronoi - Green's theorem per il calcolo del centroide -
template<typename T>
Vector2<T> computeCentroid(const Diagram<T>& diagram)
{
    //Funzione calcolo centroide del mainSite (site 0) o equivalentemente della mainFace (face 0 associata al site 0)
    auto area = static_cast<T>(0.0);                        //inizializzazione area (double)
    auto centroid = Vector2<T>();                           //inizializzazione variabile centroide
    auto halfEdge = diagram.getFace(0)->outerComponent;     //prendiamo l'half-edge puntato dalla faccia
    //Compute centroid of the face
    do
    {
        auto det = halfEdge->origin->point.getDet(halfEdge->destination->point);    //prodotto vettoriale vettore origine e vettore destinazione dell'half-edge considerato
        area += det;                                                                //contributo al calcolo dell'area del poligono (singolo termine della sommatoria)
        centroid += (halfEdge->origin->point + halfEdge->destination->point) * det; //contributo al calcolo del centroide (singolo termine della sommatoria)
        halfEdge = halfEdge->next;                                                  //passaggio all'half-edge successivo della face 0
    } while(halfEdge != diagram.getFace(0)->outerComponent);    //ciclo do-while continua fino ad esaurimento degli half-edge della faccia considerato
    area *= 0.5;                                                //area del poligono/cella centrale del diagramma
    centroid *= 1.0 / (6.0 * area);                             //centroide del poligono/cella centrale del diagramma

    return centroid;    //centroide = spostamento locale rispetto a posizione corrente {0.0,0.0}
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//----------------------------- Compute centroids - Gauss Distribution - Line Integration Method ------------------------------------
//Funzioni utili al calcolo dei centroidi nel caso di densit� non uniforme (Gaussian Density Function by Line Integration, vedi paper omonimo)

//Funzione per calcolo integrale definito
//Metodo Simpson (++ precisione) --> approssima in sottintervalli ciascuno dei quali � definito da una parabola (curva del secondo grado)
template<typename Function>
double computeIntegral(double a, double b, std::size_t numBins, Function f)
{
    double step = (b-a)/numBins;    //numBins: numero di sottointervalli, step: ampiezza sottointervallo
    double s = a;                   //a: primo estremo di integrazione
    double integral = 0;
    while(s < b){                   //b: secondo estremo di integrazione
        integral += step*(f(s) + f(s+step) + 4*f(0.5*(s+s+step)))/6.;  //integrale = somma delle aree dei singoli blocchetti/intervalli
        s += step;  //passaggio ad intervallo successivo
    }
    return integral;
}

//Funzione f(s) integranda della massa della cella
double integranda_mass(double s, Vector2<double> currentVertex, Vector2<double> nextVertex, Vector2<double> local_p_t, double var)
{
    double erf_term = std::erf(((nextVertex.x - currentVertex.x)*s + currentVertex.x - local_p_t.x)/((std::sqrt(2))*var));
    double pow_term = std::pow(((nextVertex.y - currentVertex.y)*s + currentVertex.y - local_p_t.y)/var, 2);
    double exp_term = std::exp(-0.5*pow_term);

    return erf_term*exp_term;
}
//Funzione f(s) integranda della coordinata x del centroide
double integranda_centroid_x(double s, Vector2<double> currentVertex, Vector2<double> nextVertex, Vector2<double> local_p_t, double var)
{
    double erf_term = std::erf(((nextVertex.y - currentVertex.y)*s + currentVertex.y - local_p_t.y)/((std::sqrt(2))*var));
    double pow_term = std::pow(((nextVertex.x - currentVertex.x)*s + currentVertex.x - local_p_t.x)/var , 2);
    double exp_term = std::exp(-0.5*pow_term);
    double lin_term = ((nextVertex.x - currentVertex.x)*s + currentVertex.x);

    return lin_term*exp_term*erf_term;
}
//Funzione f(s) integranda della coordinata x del centroide
double integranda_centroid_y(double s, Vector2<double> currentVertex, Vector2<double> nextVertex, Vector2<double> local_p_t, double var)
{
    double erf_term = std::erf(((nextVertex.x - currentVertex.x)*s + currentVertex.x - local_p_t.x)/((std::sqrt(2))*var));
    double pow_term = std::pow(((nextVertex.y - currentVertex.y)*s + currentVertex.y - local_p_t.y)/var , 2);
    double exp_term = std::exp(-0.5*pow_term);
    double lin_term = ((nextVertex.y - currentVertex.y)*s + currentVertex.y);

    return lin_term*exp_term*erf_term;
}

//Vettore dei centroidi non geometrici (densit� Gaussiana) in funzione dei vertici della cella e dei parametri della distribuzione (var, p_t)
template<typename T>
std::vector<Vector2<T>> computeGaussianCentroids(const std::vector<Diagram<T>>& diagrams, Vector2<T> p_t, double var)
{
    auto centroids = std::vector<Vector2<T>>();

    for(const auto& diagram : diagrams){
        centroids.push_back(computeGaussianCentroid(diagram, p_t, var));
    }

    return centroids;
}

template<typename T>
Vector2<T> computeGaussianCentroid(const Diagram<T>& diagram, Vector2<T> p_t, double var)
{
    //Calcolo p_t in coordinate locali (local_p_t);
    auto local_p_t = p_t - diagram.getGlobalPoint();
    //debug
    if (DEBUG >= 1)
    {
        std::cout << "Punto di interesse -->" << local_p_t << std::endl;
    }
    //Inizializzazioni massa e centroide
    auto mass = static_cast<T>(0.0);                        //inizializzazione double massa
    auto centroid = Vector2<T>();                           //inizializzazione centroide
    auto halfEdge = diagram.getFace(0)->outerComponent;     //prendiamo l'half-edge puntato dalla faccia
    do
    {   //MVi
        double integral = computeIntegral(0, 1, 500, [&](double s){return integranda_mass(s, halfEdge->origin->point, halfEdge->destination->point, local_p_t, var);});
        mass += integral*(halfEdge->destination->point.y - halfEdge->origin->point.y);
        //Cx
        double integral_x = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_x(s, halfEdge->origin->point, halfEdge->destination->point, local_p_t, var);});
        centroid.x += integral_x*(halfEdge->origin->point.x - halfEdge->destination->point.x);
        //Cy
        double integral_y = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_y(s, halfEdge->origin->point, halfEdge->destination->point, local_p_t, var);});
        centroid.y += integral_y*(halfEdge->destination->point.y - halfEdge->origin->point.y);

        //Passaggio all'half-edge successivo della face 0
        halfEdge = halfEdge->next;
    } while(halfEdge != diagram.getFace(0)->outerComponent);
    //Moltiplicazione per le costanti comuni
    mass *= (((std::sqrt(2*M_PI))*var)/2);
    centroid.x *= (1/mass)*(((std::sqrt(2*M_PI))*var)/2);
    centroid.y *= (1/mass)*(((std::sqrt(2*M_PI))*var)/2);

    return centroid;
}

//Vettore dei centroidi non geometrici due punti di interesse (densit� Gaussiana) in funzione dei vertici della cella e dei parametri della distribuzione (var, p_t)
template<typename T>
std::vector<Vector2<T>> compute2GaussianCentroids(const std::vector<Diagram<T>>& diagrams, const std::vector<Vector2<T>>& p_ts, const std::vector<double>& vars)
{
    //Vettore dei centroidi gaussiani
    auto centroids = std::vector<Vector2<T>>();

    //Per ogni diagramma del vettore dei diagrammi � necessario calcolare il centroide del sito centrale main (site 0)
    for(const auto& diagram : diagrams){
        centroids.push_back(compute2GaussianCentroid(diagram, p_ts, vars));  //inserimento nel vettore dei centroidi
    } //dopodich� si prosegue con il diagramma successivo

    return centroids;
}

template<typename T>
Vector2<T> compute2GaussianCentroid(const Diagram<T>& diagram, const std::vector<Vector2<T>>& p_ts, const std::vector<double>& vars)
{
    //Vettore dei punti interesse in coordinate locali
    auto local_p_ts = std::vector<Vector2<T>>();
    //Calcolo punti p_t in coordinate locali (local_p_t);
    for(const auto& p_t : p_ts){
        auto local_p_t = p_t - diagram.getGlobalPoint();        //trasformazione
        local_p_ts.push_back(local_p_t);                        //salvataggio nel vettore
        //debug
        if (DEBUG >= 1)
        {
            std::cout << "Punto di interesse -->" << local_p_t << std::endl;
        }
    }

    //Inizializzazioni massa e centroide
    auto mass = static_cast<T>(0.0);                        //inizializzazione double massa
    auto centroid = Vector2<T>();                           //inizializzazione centroide
    auto halfEdge = diagram.getFace(0)->outerComponent;     //prendiamo l'half-edge puntato dalla faccia
    for(std::size_t i=0; i<local_p_ts.size(); ++i){
        do
        {   //MVi
            double integral = computeIntegral(0, 1, 500, [&](double s){return integranda_mass(s, halfEdge->origin->point, halfEdge->destination->point, local_p_ts[i], vars[i]);});
            mass += integral*(halfEdge->destination->point.y - halfEdge->origin->point.y);
            //Cx
            double integral_x = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_x(s, halfEdge->origin->point, halfEdge->destination->point, local_p_ts[i], vars[i]);});
            centroid.x += integral_x*(halfEdge->origin->point.x - halfEdge->destination->point.x);
            //Cy
            double integral_y = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_y(s, halfEdge->origin->point, halfEdge->destination->point, local_p_ts[i], vars[i]);});
            centroid.y += integral_y*(halfEdge->destination->point.y - halfEdge->origin->point.y);

            //Passaggio all'half-edge successivo della face 0
            halfEdge = halfEdge->next;
        } while(halfEdge != diagram.getFace(0)->outerComponent);
        //Moltiplicazione per le costanti comuni
        mass *= (((std::sqrt(2*M_PI))*vars[i])/2);
        centroid.x *= (((std::sqrt(2*M_PI))*vars[i])/2);
        centroid.y *= (((std::sqrt(2*M_PI))*vars[i])/2);
    }
    centroid.x *= (1/mass);
    centroid.y *= (1/mass);

    return centroid;
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//------------------------------- Calcolo centroide con Distribuzione Gaussiana discretizzando l'area ----------------------------------------------------------------------------------

//Jordan curve theorem:
//per verificare se un punto si trova dentro o fuori un poligono
//traccio una riga orizzontale (aumentando x e tenendo fissa y) che esce dal punto in questione
//se la riga interseca il poligono un numero pari di volte allora si trova FUORI dal poligono
//se la riga interseca il poligono un numero dispari di volte allora si trova DENTRO il poligono
//la variabile bool in viene cambiata ogni volta che vi � una intersezione con il poligono
template<typename T>
bool inPolygon(const Diagram<T> &polygon, Vector2<T> point)
{
    bool in = false;
    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //partendo da un lato ripercorro tutto il perimentro del poligono, vertice per vertice verificando la presenza di intersezioni
    do{
        if (((halfedge->origin->point.y > point.y) != (halfedge->destination->point.y > point.y))
             && (point.x < (halfedge->destination->point.x - halfedge->origin->point.x)
                * (point.y - halfedge->origin->point.y) / (halfedge->destination->point.y - halfedge->origin->point.y) + halfedge->origin->point.x)){
            in = !in;
        }
        halfedge = halfedge->next;

    } while (halfedge != seed->face->outerComponent);

    return in;
}

// template<typename T>
// bool inObstacle(Box<T> ObstacleBox, Vector2<T> point)
// {
//     bool in = ObstacleBox.contains(point);
//     return in;
// }

template<typename T>
bool inObstacles(std::vector<Box<T>> ObstacleBoxes, Vector2<T> point)
{
    // std::cout << "------------- DEBUG: -------------- \n";
    // std::cout << "x: " << point.x << ", y: " << point.y << std::endl;

    bool in = false;
    std::vector<bool> boolVector(ObstacleBoxes.size());      // vettore di bool con stessa dimensione di ObstacleBoxes
    for (int i=0; i<ObstacleBoxes.size(); ++i)
    {
        // std::cout << "---- Obstacle ----  \n";
        // std::cout << "bottom left: " << ObstacleBoxes[i].left << ObstacleBoxes[i].bottom << std::endl;
        // std::cout << "top right: " << ObstacleBoxes[i].right << ObstacleBoxes[i].top << std::endl;
        boolVector[i] = ObstacleBoxes[i].contains(point);
    }
    
    // in = false solo se tutti false
    for (int i=0; i<boolVector.size(); ++i)
    {
        in = in || boolVector[i];
    }

    return in;
}

//Funzione creata per obiettivi di debug nel calcolo dell'area
template<typename T>
double computePolygonArea(const Diagram<T> &polygon, Vector2<T> pt_mean, T var, double discretize_precision = 1.0/100.0){
    //Calcolo p_t in coordinate locali (local_p_t);
    auto local_pt_mean = pt_mean - polygon.getGlobalPoint();

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    double x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    double x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    double y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    double y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    halfedge = halfedge->next;

    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);


    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    double dx = (x_sup - x_inf)/2.0 * discretize_precision;
    double dy = (y_sup - y_inf)/2.0 * discretize_precision;
    double dA = dx*dy;
    double A = 0;

    for (double i = x_inf; i <= x_sup; i=i+dx)
    {
        for (double j = y_inf; j <= y_sup; j=j+dy)
        {
            //std::cout<<"j value :: "<<j<<"\n"<<std::scientific;
            bool in = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            if (in)
            {
                A = A + dA*gauss_pdf(local_pt_mean, var, Vector2<double> {i,j});
            }
        }
    }
    return A;
}

//Calcolo del centroide discretizzando l'area del poligono
//viene creato un rettangolo circoscritto al poligono e suddiviso in tanti elementi quanti definito da parametro
//ogni elemento viene considerato se appartenente al poligono (inPolygon)
//per ogni elemento viene calcolato il valore della PDF nel suo baricentro
template<typename T>
Vector2<T> computePolygonCentroid(const Diagram<T> &polygon, std::vector<Vector2<T>> pt_means, std::vector<T> vars, double discretize_precision = 1.0/100.0){
    //Calcolo p_t in coordinate locali (local_p_t);
    for (long unsigned int i = 0; i < pt_means.size(); ++i)
    {
        pt_means[i] = pt_means[i] - polygon.getGlobalPoint();
        //auto local_pt_mean = pt_mean - polygon.getGlobalPoint();
    }

    //DEBUG
    //std::cout<<"gaussian relative position ::: "<<local_pt_mean<<"\n"<<std::scientific;

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    double x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    double x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    double y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    double y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    halfedge = halfedge->next;

    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);

    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    double dx = (x_sup - x_inf)/2.0 * discretize_precision;
    double dy = (y_sup - y_inf)/2.0 * discretize_precision;
    double dA = dx*dy;
    double A = 0;
    double Cx = 0, Cy = 0;

    for (double i = x_inf; i <= x_sup; i=i+dx)
    {
        for (double j = y_inf; j <= y_sup; j=j+dy)
        {
            //std::cout<<"j value :: "<<j<<"\n"<<std::scientific;
            bool in = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            if (in)
            {
                double dA_pdf;
                if (vars.size() <= 1)
                {
                    dA_pdf = dA*gauss_pdf(pt_means[0], vars[0], Vector2<double> {i,j});
                } else {
                    dA_pdf = dA*multiple_gauss_pdf(pt_means, vars, Vector2<double> {i,j});
                }
                A = A + dA_pdf;
                Cx = Cx + i*dA_pdf;
                Cy = Cy + j*dA_pdf;
            }
        }
    }
    Cx = Cx / A;
    Cy = Cy / A;

    if (DEBUG >= 1)
    {
        std::cout<<std::scientific<<" ------------------------ Area : "<<A<<std::endl;
    }

    Vector2<double> C = {Cx, Cy};
    return C;
}

// Funzione per calcolare il centroide con distribuzione gaussiana repulsiva
template<typename T>
Vector2<T> computeRepulsivePolygonCentroid(const Diagram<T> &polygon, std::vector<Vector2<T>> pt_means, std::vector<T> vars, std::vector<Box<T>> ObstacleBoxes, double discretize_precision = 1.0/100.0){
    //Calcolo p_t in coordinate locali (local_p_t);
    for (long unsigned int i = 0; i < pt_means.size(); ++i)
    {
        pt_means[i] = pt_means[i] - polygon.getGlobalPoint();
        //auto local_pt_mean = pt_mean - polygon.getGlobalPoint();
    }

    //DEBUG
    //std::cout<<"gaussian relative position ::: "<<local_pt_mean<<"\n"<<std::scientific;

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    double x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    double x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    double y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    double y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    halfedge = halfedge->next;

    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);

    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    double dx = (x_sup - x_inf)/2.0 * discretize_precision;
    double dy = (y_sup - y_inf)/2.0 * discretize_precision;
    double dA = dx*dy;
    double A = 0;
    double Cx = 0, Cy = 0;

    std::cout << "Coordinate sito: " << polygon.getGlobalPoint() << std::endl;

    for (double i = x_inf; i <= x_sup; i=i+dx)
    {
        for (double j = y_inf; j <= y_sup; j=j+dy)
        {   
            // Rielaborazione ostacoli in coord locali
            std::vector<Box<double>> localObstacleBoxes(ObstacleBoxes.size());
            for (int i=0; i<ObstacleBoxes.size(); ++i)
            {
                localObstacleBoxes[i] = reworkObstacle(ObstacleBoxes[i], polygon.getGlobalPoint());
            }

            bool inArea = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            bool inObstacle = inObstacles(localObstacleBoxes, Vector2<double> {i+dx, j+dy});
            if (inArea && !inObstacle)
            {
                double dA_pdf;
                if (vars.size() <= 1)
                {
                    dA_pdf = dA*repulsive_gauss_pdf(pt_means[0], vars[0], Vector2<double> {i,j});
                } else {
                    dA_pdf = dA*multiple_repul_gauss_pdf(pt_means, vars, Vector2<double> {i,j});
                }
                A = A + dA_pdf;
                Cx = Cx + i*dA_pdf;
                Cy = Cy + j*dA_pdf;
            }
        }
    }
    Cx = Cx / A;
    Cy = Cy / A;

    if (DEBUG >= 1)
    {
        std::cout<<std::scientific<<" ------------------------ Area : "<<A<<std::endl;
    }

    Vector2<double> C = {Cx, Cy};
    
    return C;
}

template<typename T>
std::vector<Vector2<T>> computeDiagramsCentroids(const std::vector<Diagram<T>>& diagrams, std::vector<Vector2<T>> pt_means, std::vector<T> vars, double discretize_precision = 1.0/100.0){
    //Vettore dei centroidi gaussiani
    std::vector<Vector2<T>> centroids;

    for (const auto &diagram : diagrams)
    {
        auto C = computePolygonCentroid(diagram, pt_means, vars, discretize_precision);
        centroids.push_back(C);
    }
    return centroids;
}

template<typename T>
std::vector<Vector2<T>> computeRepulsiveDiagramsCentroids(const std::vector<Diagram<T>>& diagrams, std::vector<Vector2<T>> pt_means, std::vector<T> vars, std::vector<Box<T>> ObstacleBoxes, double discretize_precision = 1.0/100.0){
    //Vettore dei centroidi gaussiani
    std::vector<Vector2<T>> centroids;

    for (const auto &diagram : diagrams)
    {
        auto C = computeRepulsivePolygonCentroid(diagram, pt_means, vars, ObstacleBoxes, discretize_precision);
        centroids.push_back(C);
    }
    return centroids;
}

//funzione per calcolare il valore di un punto data una distribuzione a forma di gaussiana
template<typename T>
double gauss_pdf(Vector2<T> mean_pt, T var, Vector2<T> pt){
    double temp = (std::pow((pt.x-mean_pt.x),2.0) + std::pow((pt.y-mean_pt.y),2.0)) / (2.0 * std::pow(var,2.0));
    return std::exp(-temp);
}

//funzione per calcolare il valore di un punto data una serie di distribuzioni a forma di gaussiana
template<typename T>
double multiple_gauss_pdf(std::vector<Vector2<T>> means_pt, std::vector<T> vars, Vector2<T> pt){
    double val = 0;
    for (long unsigned int i = 0; i < vars.size(); ++i)
    {
        val = val + gauss_pdf(means_pt[i], vars[i], pt);
    }
    return val;
}

//funzione per calcolare il valore di un punto data una distribuzione gaussiana repulsiva troncata a 1
template<typename T>
double repulsive_gauss_pdf(Vector2<T> mean_pt, T var, Vector2<T> pt){
    double temp = (std::pow((pt.x-mean_pt.x),2.0) + std::pow((pt.y-mean_pt.y),2.0)) / (2.0 * std::pow(var,2.0));
    double temp2 = std::exp(-temp);
    double temp3 = 1/temp2 - 1;                         // Ribalto e abbasso di 1 per avere il minimo in 0                
    double val = 0;
    if (temp3 > 1)                                      // Taglio i valori > 1
    {
        val = 1;
    } else
    {
        val = temp3;
    }
    return val;
}

//funzione per calcolare il valore di un punto data una serie di distribuzioni gaussiane repulsive
template<typename T>
double multiple_repul_gauss_pdf(std::vector<Vector2<T>> means_pt, std::vector<T> vars, Vector2<T> pt){
    double tmp = 0;
    double val = 0;
    for (long unsigned int i = 0; i < vars.size(); ++i)
    {
        tmp = tmp + repulsive_gauss_pdf(means_pt[i], vars[i], pt);
    }
    tmp = tmp - (vars.size()-1);
    if (tmp < 0)
    {
        val = 0;
    } else
    {
        val = tmp;
    }
    return val;
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//------------------------------------ Update Robots positions - Simulate Robots motion to centroids --------------------------------

//Se robot conosce la sua posizione globale iniziale magari la funzione updatepoints potr� ragionare su mGlobalPoint (ed eventualmente su un mCentroid)
//tutte le seguenti funzioni in pratica andranno spostate in Diagram.h poich� diventeranno parte integrante del singolo diagramma
//Aggiornamento vettore globale "points" (da utilizzare dopo calcolo vettore centroidi per poter ricalcolare i diagrammi con le nuove posizioni)
template<typename T>
std::vector<Vector2<T>> updatepoints(const std::vector<Diagram<T>>& diagrams, const std::vector<Vector2<T>>& centroids)
{
    auto updt_points = std::vector<Vector2<T>>();
    auto global_points = std::vector<Vector2<T>>();

    //Raccogliamo gli mGlobalPoint correnti in un vettore
    for(const auto& diagram : diagrams){
        global_points.push_back(diagram.getGlobalPoint());
    }
    //Aggiungiamo al valore points[i] originario il valore del centroide[i](ovvero lo spostamento locale rispetto a {0.0,0.0})
    for(std::size_t i=0; i<global_points.size(); ++i){
        auto updt_point = global_points[i] + centroids[i];
        updt_points.push_back(updt_point);
    }

    return updt_points;
}

//Update posizione globale
template<typename T>
Vector2<T> updateGlobalPoint(const Diagram<T>& diagram, const Vector2<T>& centroid)
{
    //Aggiungiamo al valore point originario il valore del centroide(ovvero lo spostamento locale rispetto a {0.0,0.0})
    auto updt_point = diagram.getGlobalPoint() + centroid;

    return updt_point;
}
//***********************************************************************************************************************************

#endif // VORONOI_H_INCLUDED
