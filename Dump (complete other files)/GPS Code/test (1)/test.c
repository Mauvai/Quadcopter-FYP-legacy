#include <stdio.h>

extern _Driver _SimpleSerialDriver;
extern _Driver _FileDriver;

_Driver *_driverlist[] = {
  &_SimpleSerialDriver,
  &_FileDriver,
  NULL
};

void error_exit(char *fname)
{
    printf("Could not open %s\n", fname);
    exit(1);
}

int main()
{
    int i;
    float x = 2015;
    FILE *outfile;
    FILE *infile;
    
    outfile = fopen("testfile.dat", "a+");
    if (!outfile) error_exit("testfile.dat");
    for (i = 0; i < 4; i++, x++) fwrite(&x, 1, 4, outfile);
    fclose(outfile);
    
    infile = fopen("testfile.dat", "r");
    if (!infile) error_exit("testfile.dat");
    while (fread(&x, 1, 4, outfile)) printf("%f\n", x);
    fclose(outfile);
    
    return 0;
}
