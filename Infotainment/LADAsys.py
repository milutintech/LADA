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


# --- Custom CSS ---
# Custom CSS to center content vertically in Streamlit
vertical_center_css = """
<style>
div.stButton > button:first-child {
    display: block;
    margin: 0 auto;
}
.flex-container {
    display: flex;
    flex-direction: column;
    justify-content: center;
    height: 500px;
}
</style>
"""




background_image = """
<style>
[data-testid="stAppViewContainer"] > .main {
    background-image: url("https://img.freepik.com/free-photo/abstract-colorful-splash-3d-background-generative-ai-background_60438-2503.jpg?w=2000&t=st=1709903923~exp=1709904523~hmac=da587f2827b648b31e9ab3bc19717c177231c0571a41009fe768867026442876");
    background-size: 100vw 100vh;  # This sets the size to cover 100% of the viewport width and height
    background-position: center;  
    background-repeat: no-repeat;
}
</style>
"""

st.markdown(background_image, unsafe_allow_html=True)

st.markdown(
    """
<style>
button {
    height: auto;
    padding-top: 100px !important;
    padding-bottom: 100px !important;
}
</style>
""",
    unsafe_allow_html=True,
)
# --- Main ---



# --- Title ---




# Inject custom CSS with the above styles
st.markdown(vertical_center_css, unsafe_allow_html=True)

# Use a column layout with a flex container to center content vertically
col1, col2, col3 = st.columns([1,4,1])

with col2:
    st.markdown('<div class="flex-container">', unsafe_allow_html=True)
    # Place your content here; for demonstration, a button is used
    st.button("Click Me!")
    st.markdown('</div>', unsafe_allow_html=True)