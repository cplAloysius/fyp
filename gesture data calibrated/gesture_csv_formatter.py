infile = input("input file name:")
output_filename = infile[:-3] + "csv"
with open(infile, 'r') as infile, open(output_filename, 'w') as outfile:
    for line in infile:
        if 'starting' in line:
            continue
        else:
            outfile.write(line)

print(f'Converted data saved to {output_filename}')
