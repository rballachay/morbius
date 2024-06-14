import numpy as np
import string
import re

def remove_punctuation_whitespace(text):
    text=text.lower().strip()
    text=text.replace('-',' ')
    # Remove punctuation
    text = text.translate(str.maketrans('', '', string.punctuation))
    return text

def calculate_wer(reference, hypothesis):
    ref_words = remove_punctuation_whitespace(reference).split()
    hyp_words = remove_punctuation_whitespace(hypothesis).split()
    
    # Create a matrix to store the edit distances
    d = np.zeros((len(ref_words) + 1, len(hyp_words) + 1))
    
    # Initialize the matrix
    for i in range(len(ref_words) + 1):
        d[i][0] = i
    for j in range(len(hyp_words) + 1):
        d[0][j] = j
    
    # Fill in the matrix
    for i in range(1, len(ref_words) + 1):
        for j in range(1, len(hyp_words) + 1):
            if ref_words[i - 1] == hyp_words[j - 1]:
                d[i][j] = d[i - 1][j - 1]
            else:
                substitution = d[i - 1][j - 1] + 1
                insertion = d[i][j - 1] + 1
                deletion = d[i - 1][j] + 1
                d[i][j] = min(substitution, insertion, deletion)
    
    wer = d[len(ref_words)][len(hyp_words)] / len(ref_words)
    return wer, len(ref_words)