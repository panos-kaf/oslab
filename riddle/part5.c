#include <fcntl.h>
#include <unistd.h>

int main(){
	char* argv[]={"./riddle",NULL};
	dup2(0,99);
	execv("./riddle",argv);
}

