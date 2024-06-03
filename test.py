from src.language.transcribe import listen_transcribe

if __name__ == "__main__":
    # Pass FILENAME as a parameter
    results = listen_transcribe()
    print(results)
