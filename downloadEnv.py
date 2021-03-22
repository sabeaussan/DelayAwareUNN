import os
import platform
import zipfile
import gdown

def get_assets():
        asset_name = get_asset_name(platform.system())
        print('downloading', "environments ...")
        gdown.download("https://drive.google.com/uc?id="+asset_name,"env.zip", False)
        zip_ref = zipfile.ZipFile('env.zip', 'r')
        print('unpacking ...')
        zip_ref.extractall()
        zip_ref.close()
        print("Unpacked !")
    
def get_asset_name(plateform):
    if platform.system().lower() == "darwin" :
        asset_name = "1qGKwRYUKigLc9mJvc5aH-OJNvBpOGNY0"
    if platform.system().lower() == "linux" :
        asset_name = "1vGnTL0fPJXSwM07_vodYgrxejzQDZ6iB"
    if platform.system().lower() == "windows" :
        asset_name = "1WtzCnddrWWJwCyBl0d2t5FpkIHclz79k"
    return asset_name

get_assets()