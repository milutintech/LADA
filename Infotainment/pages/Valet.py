# --- Sources ---
# https://streamlit-emoji-shortcodes-streamlit-app-gwckff.streamlit.app/

# --- Setup ---
# pip install Streamlit
# pip install streamlit-option-menu

# --- Libs ---
import streamlit as st
from streamlit_option_menu import option_menu
import base64

# --- Colors ---
Grey100 = "#F5F5F5"
Grey200 = "#EEEEEE"
Grey300 = "#E0E0E0"
Grey400 = "#BDBDBD"
Grey500 = "#9E9E9E"
Grey600 = "#757575"
Grey700 = "#616161"
Grey800 = "#424242"
Grey900 = "#212121"

MGrey   = "#373b3e"
MAnthrazit =  "#46525a"
MBlue   = "#005ca8"

# --- Constants ---


st.set_page_config(page_title="LADAsys", page_icon=":blue_car:", layout="wide")

# --- Variables ---


# --- Functions ---
def get_base64(bin_file):
    with open(bin_file, 'rb') as f:
        data = f.read()
    return base64.b64encode(data).decode()


def set_background(png_file):
    bin_str = get_base64(png_file)
    page_bg_img = '''
    <style>
    .stApp {
    background-image: url("data:image/png;base64,%s");
    background-size: cover;
    }
    </style>
    ''' % bin_str
    st.markdown(page_bg_img, unsafe_allow_html=True)

set_background('Images/Background.png')

# --- Custom CSS ---
# Custom CSS to center content vertically in Streamlit
vertical_center_css = """
<style>
div.stButton > button:first-child {
    display: block;
    margin: 0 auto;
    width: 200px;
}
.flex-container {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    height: 15vh;
}
</style>
"""
#height: 300px;




st.markdown(
    """
<style>
    button {
        height: auto;
        padding-top: 200px !important;
        padding-bottom: 200px !important;
        padding-left: 50px !important;
        padding-right: 50px !important;
    }
    .reportview-container {
            margin-top: -2em;
        }
        #MainMenu {visibility: hidden;}
        .stDeployButton {display:none;}
        footer {visibility: hidden;}
        #stDecoration {display:none;}
</style>
""",
    unsafe_allow_html=True,
)

st.markdown(vertical_center_css, unsafe_allow_html=True)

# --- Main ---



# --- Title ---
st.title("Valet Mode")

st.markdown(
    """
    <style>
        .stProgress > div > div > div > div {
            background-color: green;
        }
    </style>""",
    unsafe_allow_html=True,
)

# Use a column layout with a flex container to center content vertically
col0, col1, col2, col3, col4 = st.columns([1, 5, 1, 2, 1])

with col1:
    st.markdown('<div class="flex-container">', unsafe_allow_html=True)
    BatNow      = 42
    StromNow    = 20
    TempNow     = 60
    VoltNow     = 400

    latest_iteration = st.empty()
    latest_iteration.text(f'Ladezustand: {BatNow}%')
    st.progress(BatNow)

    st.text(f"Akteuller Ladestrom: {StromNow}A")
    st.text(f"Akteulle Temperatur: {TempNow}°C")
    st.text(f"Aktuelle Spannung:   {VoltNow}V")
    st.markdown('</div>', unsafe_allow_html=True)

with col3:
    st.markdown('<div class="flex-container">', unsafe_allow_html=True)
    # Place your content here; for demonstration, a button is used
    if st.button("Exit Valet Mode"):
        st.switch_page('LADAsys.py')
    st.markdown('</div>', unsafe_allow_html=True)

