# --- Setup ---
# pip install Streamlit
# pip install streamlit-option-menu

# --- Libs ---
import streamlit as st
from streamlit_option_menu import option_menu

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


def set_png_as_page_bg():
    bin_str = get_base64_of_bin_file(Images/Background.png)
    page_bg_img = '''
    <style>
    body {
    background-image: url("data:image/png;base64,%s");
    background-size: cover;
    }
    </style>
    ''' % bin_str
    
    st.markdown(page_bg_img, unsafe_allow_html=True)
    return

# --- Variables ---


# --- Functions ---


# --- Main ---
st.set_page_config(page_title="LADAsys", page_icon=":RacingCar:", layout="wide")

# --- Title ---


selected = option_menu(
    menu_title=None,
    options=["Home", "Run Test", "Test Setup"],
    icons=["house", "rocket-takeoff", "gear"],
    orientation="horizontal"
)


st.write("---")
#st.side_bg = "Images/Background.png" #, width=200)

page_bg_img = '''
<style>
body {
background-image: "Images/Background.png";
background-size: cover;
}
</style>
'''

st.markdown(page_bg_img, unsafe_allow_html=True)

# --- Menu ---
# -- Home --