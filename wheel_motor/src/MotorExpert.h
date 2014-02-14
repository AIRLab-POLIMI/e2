#include <sys/time.h>
#include <stdlib.h> 
#include <stdio.h>

#include "GianoActuationManager.h"


class MotorExpert:public GianoActuationManager
{
public:
    
    //Costruttore
    MotorExpert();
    
    //Deve diventare la funzione che restituisce i valori
    //che restituiscono gli encorer
    void getOdometryData();
    
    //Invia comandi al robot. La funzione si occupa anche di eseguire la conversione
    //dei dati. Da TanSpeed e RotSpeed vengono calcolati i tre valori che devono
    //essere inviati ai motori tramite seriale
    void actuations(int tanSpeed, int rotSpeed);
    
    //Distruttore
    ~MotorExpert ();
    
private:    
    int degree;
    int wait_to_read_compass;
    //void AddOdometryData(string name, float value);
    //void AddCompassData(string name, int value);
    
};
