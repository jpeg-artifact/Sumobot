import requests

def read_text_from_url(url):
    try:
        response = requests.get(url)
        response.raise_for_status()  # Kontrollera om förfrågan var framgångsrik

        text_content = response.text
        return text_content
    except requests.exceptions.RequestException as e:
        print("Det uppstod ett fel vid hämtning av URL:", e)
        return None

# Ange din URL här
url = "https://track.ssis.nu/last/T10"

text_content = read_text_from_url(url)
if text_content:
    print("Textinnehållet från URL:")
    print(text_content)
