#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/wait.h>

void setup_tier2(){
	long int* addr;
        int fd = open("./temp",O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
        long int pid = (long int) getpid();
        write(fd,&pid,sizeof(long int));
        addr = (long int*)mmap((void*)0x6042000,sizeof(long int),PROT_READ | PROT_WRITE, MAP_FIXED | MAP_SHARED, fd,0);
        //printf("%p\n",addr);
        //printf("%li\n",*addr);
	return;
}

