#ifndef COPY_FILE_H
#define COPY_FILE_H

#include <fstream>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include<sys/types.h>

using namespace std;

static bool copyFile(const char *sourceFileNameWithPath, const char *targetFileNameWithPath)
{
	FILE *fpR, *fpW;

	int buffer_size = 512;
	char buffer[buffer_size];
    memset(buffer, 0, buffer_size);

    if (fopen(targetFileNameWithPath, "r") == NULL)
    {
		char *file_name = strrchr(const_cast<char*>(targetFileNameWithPath),'/');
		if (file_name == NULL)
		{
			printf("Invalid file: %s\n", targetFileNameWithPath);
			return false;
		}

		int target_file_path_length = strlen(targetFileNameWithPath) - strlen(file_name);
		char* target_file_path = new char[target_file_path_length + 1];
		if (target_file_path != NULL)
		{
			target_file_path[target_file_path_length] = 0;
			memcpy(target_file_path, targetFileNameWithPath, target_file_path_length);

			if (access(target_file_path, 0) < 0) 
			{
				mkdir(target_file_path, 0777);
			}

			delete [] target_file_path;
			target_file_path = NULL;
		}

        fpR = fopen(targetFileNameWithPath, "wb+");
        fpW = fopen(sourceFileNameWithPath, "r");

        if ( fpR == NULL || fpW == NULL)
        {
            return false;
     	}

		fseek(fpW,	0, 0);
		fseek(fpR,	0, 0);

		while (feof(fpW) == 0)
		{
			buffer[0] = '\0';
			fgets(buffer, buffer_size, fpW);
			fputs(buffer, fpR);
		}

		fclose(fpR);
		fclose(fpW);
    }

	return true;
}

#endif
