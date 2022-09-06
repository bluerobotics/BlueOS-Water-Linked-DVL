import urllib
import requests
import json
from loguru import logger


def request(url):
    """
    Sends a request to "url" and returns text result if successfull, None otherwise
    """
    try:
        return urllib.request.urlopen(url, timeout=1).read().decode()
    except Exception as error:
        logger.warning("Error in request: {0}: {1}".format(url, error))
        return None


def post(url, data):
    """
    Sends a POST request to "url" with "data" payload. returns response text if successful,
    None otherwise
    """
    try:
        jsondata = data
        jsondataasbytes = jsondata.encode('ascii')  # data should be bytes
        req = urllib.request.Request(url, data)
        req.add_header('Content-Type', 'application/json')

        with urllib.request.urlopen(req, jsondataasbytes) as response:
            return response.read()

    except Exception as error:
        logger.warning(f"Error in request: {url}: {error}")
        logger.warning(data)
        return None
