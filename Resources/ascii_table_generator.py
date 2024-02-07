#! /usr/bin/env python

import csv

def generate_ascii_table():
  """Generates an ASCII table with characters and their corresponding decimal and hexadecimal values.

  Returns:
    A list of lists representing the ASCII table data.
  """

  table = []
  for i in range(32, 127):
    char = chr(i)
    decimal = str(i)
    hexadecimal = hex(i)[2:].upper()
    table.append([char, decimal, hexadecimal])

  return table

def save_to_csv(data, filename):
  """Saves the given data to a CSV file.

  Args:
    data: A list of lists representing the data to save.
    filename: The name of the CSV file to save to.
  """

  with open(filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Character", "Decimal", "Hexadecimal"])
    writer.writerows(data)

if __name__ == "__main__":
  table_data = generate_ascii_table()
  save_to_csv(table_data, "ascii_table.csv")
  print("ASCII table saved to ascii_table.csv")
