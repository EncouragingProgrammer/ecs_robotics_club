// js/scripts.js

// Toggle the navigation menu for responsive design
function toggleMenu() {
  var topnav = document.querySelector(".topnav");
  if (topnav.className === "topnav") {
    topnav.className += " responsive";
  } else {
    topnav.className = "topnav";
  }
}
