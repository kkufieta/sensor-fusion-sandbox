import os
import sys


def edit_subtitles(directory_in_str):
    directory = os.fsencode(directory_in_str)
    filenames = ([os.fsdecode(file) for file in os.listdir(directory)])
    filenames = ["0" + filename if filename[1] ==
                 " " else filename for filename in filenames]

    os.mkdir(directory + os.fsencode('/processed'))
    all_subtitles = "\n-------\n"

    for filename in sorted(filenames):
        if not filename.endswith(".srt"):
            print("Error: This is not a .srt file!")
        else:
            read_file_name = filename[1:] if filename[0] == "0" else filename
            f = open(directory_in_str + "/" + read_file_name, 'r')
            contents = ""
            i = 1
            for line in f:
                if (line == "\n"):
                    i = 0
                if (i > 2):
                    contents += line
                i += 1
            f.close()
            # Save all subtitles to a single string and later single file
            all_subtitles += contents
            all_subtitles += "\n-------\n"

            # Save results
            f = open(directory_in_str + "/processed/" + filename, 'w')
            f.write(contents)
            f.close()
    f = open(directory_in_str + "/processed/all_subtitles", 'w')
    f.write(all_subtitles)
    f.close()


if __name__ == "__main__":
    if (len(sys.argv) < 2):
        print("You have to specify the subtitles directory")
    else:
        edit_subtitles(sys.argv[1])
