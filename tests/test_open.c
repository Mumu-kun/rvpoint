#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

int main() {
    int fd = open("test.txt", O_RDONLY);
    if (fd == -1) {
        perror("open failed");
        return 1;
    }
    char buf[10];
    int n = read(fd, buf, 5);
    write(1, buf, n);
    close(fd);
    return 0;
}
