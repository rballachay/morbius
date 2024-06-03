from src.language.transcribe import listen_transcribe
import asyncio

if __name__ == "__main__":
    # Pass FILENAME as a parameter
    results = asyncio.run(listen_transcribe())
    print(results)
