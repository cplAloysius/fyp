import csv

# Open the original CSV file
with open('raw_both1.rtf', 'r') as infile:
    reader = csv.reader(infile)
    
    # Read the first row (header) and the rest of the rows
    header = next(reader)
    rows = list(reader)

# Separate the columns based on the header
columns1 = header[:3]
columns2 = header[3:]

rows1 = [row[:3] for row in rows]
rows2 = [row[3:] for row in rows]

# Write the first CSV file
with open('raw_accel1.csv', 'w', newline='') as file1:
    writer = csv.writer(file1)
    writer.writerow(columns1)
    writer.writerows(rows1)

# Write the second CSV file
with open('raw_mag1.csv', 'w', newline='') as file2:
    writer = csv.writer(file2)
    writer.writerow(columns2)
    writer.writerows(rows2)

print("CSV files created successfully.")