#include <stdlib.h>
#include <unistd.h>


int main(){
	
	int pipefd1[2],pipefd2[2];
	char* argv[]={"./riddle",NULL};
	pipe(pipefd1);
	pipe(pipefd2);

	dup2(pipefd1[0],33);
	dup2(pipefd1[1],34);
	dup2(pipefd2[0],53);
	dup2(pipefd2[1],54);

	execv("./riddle",argv);
}

