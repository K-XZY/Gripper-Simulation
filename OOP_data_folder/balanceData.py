import pandas as pd
import random

def process_csv(input_file, output_file):
    # Read the input CSV file
    df = pd.read_csv(input_file)
    
    # Check if 'result' column exists
    if 'result' not in df.columns:
        raise ValueError("The input CSV file must have a 'result' column.")
    
    # Filter rows where 'result' is 1
    result_1_rows = df[df['result'] == 1]
    
    # Filter rows where 'result' is 0
    result_0_rows = df[df['result'] == 0]
    
    # Randomly sample rows with 'result' == 0 to match the count of 'result' == 1 rows
    sampled_result_0_rows = result_0_rows.sample(n=len(result_1_rows), random_state=42)
    
    # Combine both sets of rows
    combined_rows = pd.concat([result_1_rows, sampled_result_0_rows]).sample(frac=1, random_state=42)  # Shuffle
    
    # Save the combined rows to a new CSV file
    combined_rows.to_csv(output_file, index=False)
    print(f"Processed CSV saved to {output_file}")

# Example usage
input_csv = "balanceDataInput.csv"  # Replace with your input CSV file path
output_csv = "balanceDataOutput.csv"  # Replace with your desired output CSV file path
process_csv(input_csv, output_csv)
