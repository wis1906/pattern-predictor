#include "main.h"
#include "panoramaReader.h"


void main()
{
	panoramaReader* reader = new panoramaReader();
	reader->BeginPlay();
	reader->Tick();
}