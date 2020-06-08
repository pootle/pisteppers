
function liveupdates(updatename) {
    var esource = new EventSource("appupdates?updatename="+updatename);
    esource.addEventListener("message", function(e) {
            var newinfo=JSON.parse(e.data);
            if (newinfo=='kwac') {
                console.log('update nothing')
            } else {
                newinfo.forEach(function(update, idx) {
                    console.log(update[0] + ' is ' + update[1]);
                    var tempel=document.getElementById(update[0]);
                    if (tempel) {
                        tempel.innerHTML=update[1];
                    }
                });
            }
        }, false);
    esource.addEventListener("open", function(e) {
            console.log('update connection opened');
        }, false);
    esource.addEventListener("error", function(e) {
            if (e.readyState == EventSource.CLOSED) {
                console.log('update connection now closed');
            } else {
                console.log('update connection unhappy')
            }
        }, false);
}

async function appNotify(ele, pageid) {
    ele.disabled=true;
    fs="updateSetting?t="+ele.id+"&v="+ele.value+"&p="+pageid
    let response = await fetch(fs);
    if (response.ok) { // if HTTP-status is 200-299
        let resp = await response.text();
        console.log('response:' + resp)
        let msg = JSON.parse(resp)
        if (msg.OK) {
            if (ele.nodeName=='SELECT') {
                ele.innerHTML=msg.value;
            } else if (ele.nodeName=='INPUT') {
                console.log('imput node updated to '+msg.value);
                ele.value=msg.value;
            } else {
                ele.innerText=msg.value;
            }
        } else {
            alert(msg.fail)
        }
        console.log('good status from request >' + fs + '<, msg: ' + response.statusText)
    } else {
        console.log('bad status from request >' + fs + '<, msg: ' + response.statusText)
        alert("HTTP-Error: " + response.statusText);
    }
    ele.disabled=false;
}

async function appClickNotify(ele, pageid) {
    ele.disabled=true;
    let response = await fetch("updateSetting?t="+ele.id+"&v=0&p="+pageid);
    if (response.ok) { // if HTTP-status is 200-299
        let resp = await response.text();
        let msg = JSON.parse(resp)
        if (msg.OK) {
            if (ele.nodeName=='SELECT') {
                ele.innerHTML=msg.value;
            } else {
                ele.value=msg.value;
            }
        } else {
            alert(msg.fail)
        }
    } else {
        alert("HTTP-Error: " + response.status);
    }
    ele.disabled=false;
}

function livestreamflip(btnel) {
    var imele=document.getElementById("livestreamimg");
    if (imele.src.endsWith("nocam.png")                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 ) {
        imele.src="camstream.mjpg"
        btnel.innerHTML="hide livestream"
        console.log('live stream stopped')
    } else {
        imele.src="static/nocam.png"
        btnel.innerHTML="show livestream"
        console.log('live stream started')
    }
}

function detstreamflip(btnel) {
    var imele=document.getElementById("detstreamdiv");
    if (imele.style.display=="none") {
        imele.innerHTML='<img src="detstream.png" width = "640" height = "480" style="z-index:1;"/>';
        imele.style.display="block";
        btnel.innerHTML="hide detection"
    } else {
        imele.style.display="none";
        imele.innerHTML="";
        btnel.innerHTML="show detection"
    }
}

function maskeditflip(btnel) {
    var imele=document.getElementById("detstreamdiv");
}

function flipme(etag, itag) {
    var ele=document.getElementById(etag);
    var x=ele.style.display;
    var img=document.getElementById(itag);
    if (x=="none") {
        ele.style.display="";
        img.src="static/openuparrow.svg"
    } else {
        img.src="static/opendnarrow.svg"
        ele.style.display="none";
    }
}