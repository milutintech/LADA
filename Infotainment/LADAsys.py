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
}
.flex-container {
    display: flex;
    flex-direction: column;
    justify-content: center;
    height: 300px;
}
</style>
"""






st.markdown(
    """
<style>
button {
    height: auto;
    padding-top: 150px !important;
    padding-bottom: 150px !important;
    padding-left: 50px !important;
    padding-right: 50px !important;
}
</style>
""",
    unsafe_allow_html=True,
)
# --- Main ---



# --- Title ---




# Inject custom CSS with the above styles
st.markdown(vertical_center_css, unsafe_allow_html=True)

# Path to your icon image (this should be a local path or URL)
icon_path = 'Images/Lada_symbol.png'

# Create columns for the layout: one for the icon, one for the button
col1, col2 = st.columns([1, 8])

# Display the icon in the first column using Markdown
with col1:
    st.markdown(f"![icon]({icon_path})", unsafe_allow_html=True)

# Place a button in the second column
with col2:
    if st.button('Your Button Text'):
        st.write('Button clicked!')

# Use a column layout with a flex container to center content vertically
col0, col1, col2, col3, col4 = st.columns([1, 1,3,1, 1])


with col1:
    st.markdown('<div class="flex-container">', unsafe_allow_html=True)
    # Place your content here; for demonstration, a button is used
    st.button("CAR")
    #st.markdown('</div>', unsafe_allow_html=True)

with col2:
    st.markdown('<div class="flex-container">', unsafe_allow_html=True)
    # Place your content here; for demonstration, a button is used
    st.button("Charging")
    #st.markdown('</div>', unsafe_allow_html=True)

with col3:
    st.markdown('<div class="flex-container">', unsafe_allow_html=True)
    # Place your content here; for demonstration, a button is used
    st.button("Debug")
    #st.markdown('</div>', unsafe_allow_html=True)