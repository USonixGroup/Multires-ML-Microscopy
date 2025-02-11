// Copyright 2021 The MathWorks, Inc.

// For communication with MATLAB
var htmlComponentLocal
function setup(htmlComponent) {
    htmlComponentLocal = htmlComponent
    var dataFromMATLAB = JSON.parse(htmlComponent.Data);
    var slides = document.getElementsByClassName('slide-text');
    for (var i = 0; i < slides.length; i++) {
        slides[i].innerHTML = dataFromMATLAB[i] + ' <div class="slide-link" onclick="sendContentToMATLAB' + i + '()">' + dataFromMATLAB[slides.length] + '</div>';
    }
}

function sendContentToMATLAB0() {
    // '1' means slide 1
    htmlComponentLocal.Data = JSON.stringify('1')
}

function sendContentToMATLAB1() {
    // '2' means slide 2
    htmlComponentLocal.Data = JSON.stringify('2')
}

// Slideshow logic
var slideIndex = 1;
function showSlides() {
    var slides = document.getElementsByClassName('mySlides');
    if (slideIndex > slides.length) {
        slideIndex = 1;
    }
    if (slideIndex < 1) {
        slideIndex = slides.length;
    }
    for (var i = 0; i < slides.length; i++) {
        slides[i].style.display = 'none';
    }
    slides[slideIndex - 1].style.display = 'block';
    var dots = document.getElementsByClassName('slide-dot');
    for (var i = 0; i < dots.length; i++) {
        dots[i].className = dots[i].className.replace(' slide-dot-active', '');
    }
    dots[slideIndex - 1].className += ' slide-dot-active';
}
function plusSlides(n, fromInterval) {
    if (fromInterval == false && intervalId != 0) {
        clearInterval(intervalId);
        intervalId = 0;
    }
    slideIndex += n;
    showSlides();
}
function currentSlide(n, fromInterval) {
    if (fromInterval == false && intervalId != 0) {
        clearInterval(intervalId);
        intervalId = 0;
    }
    slideIndex = n;
    showSlides();
}
window.onload = showSlides;

var intervalId = setInterval(function() {
    plusSlides(1, true)
}, 8000); // 8 seconds delay
