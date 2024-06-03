from src.language import whisper


def run():
    __download_models()


def __download_models():
    for model_size in whisper.WHISPER_SIZES:
        whisper.download_whisper(model_size)


if __name__ == "__main__":
    run()
