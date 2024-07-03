;

// global variables
let start_point_x = 300;
let start_point_y = 50;
let d_divider = 25; // for lidar dists to pixels conversion
let orientation_angle = 90; // is needed to match robot's direction
var $washTypes = [];
const isNumeric = n => !isNaN(n)


function build_path() {
  $.post("/path/build", function(resp) {
      console.log(resp);
      response = resp;
      if (response.length == 0) {
          console.error("Can't get route!");
  }
  }).fail(function(err) {
      console.error(err);
  });
}

/*
* params: coordinates: array[int,int]
*/
function moveRobot(coordinates) {
    let d = d_divider;
    let pivot_x = start_point_x; // + offset_x
	let pivot_y = start_point_y; // + offset_y
	let offset_x = parseInt(coordinates[0])/d;
	let offset_y = parseInt(coordinates[1])/d;
	let angle_z = parseInt(coordinates[2]);

	// console.log("offset_x = " +offset_x+" | offset_y = "+offset_y);
	// console.log("pivot_x = " +pivot_x+" | pivot_y = "+pivot_y);
//	document.getElementById("robot-location").setAttribute("transform-origin", pivot_x+"px "+pivot_y+"px ;");
	document.getElementById("robot-location").setAttribute("transform", "translate(" + offset_x + "," + offset_y + ") rotate(" + angle_z + ","+ pivot_x + "," + pivot_y +")");
//    document.getElementById("lidar-lines").setAttribute("transform", "translate(" + offset_x + "," + offset_y + ")");

//	let lines = $(".robotenv")
//	for (let i = 0; i < lines.length; i++) {
//        lines[i].setAttribute("transform", "translate(" + offset_x + "," + offset_y + ")");
//    }
}

function convertCoords(x,y) {
  let offset = $("#layer-svg")[0].getBoundingClientRect();
  let matrix = $("#robot-location")[0].getScreenCTM();

  return [
  	(matrix.a * x) + (matrix.c * y) + matrix.e - offset.left,
    (matrix.b * x) + (matrix.d * y) + matrix.f - offset.top
  ];
}
//                      not needed function
//function updateRobotCoordinates() {
//	last_coordinates = [0,0];
//	$.post("/coords/get", {last_coords: last_coordinates}, function(resp) {
//		console.log(resp);
//		let coordinates = resp;
//		if (coordinates.length == 0) {
//			console.error("Can't get coordinates!");
//		} else {
//			moveRobot(coordinates);
//		}
//	}).fail(function(err) {
//		console.error(err);
//	});
//}

function getEnvironment() {
    $.post("/state/env/get", function(resp) {
      // console.log(resp);
//      response = resp;
      if (resp.length == 0) {
          console.error("Can't get environment data!");
  }   
      getAroundableStacks(resp["environment"],resp["lidar"],resp["front_lidar"],resp["coords"]["coords"]);
      moveRobot(resp["coords"]["coords"]);
      
  }).fail(function(err) {
      console.error(err);
  });
}


function getAroundableStacks(e,l, fl, coords) {
//	let pts = [];
	let last_coordinates = [0,0];
	// $.get("/points", {last_coords: last_coordinates}, function(resp) {
	// 	console.log(resp);
//		pts = [3500, 1200, 2400, 4500, 2000, 3000, 12000, 12000, 12000, 1110, 1400, 3200];
//        let env = getEnvironment()
        let pts = [e["frw"]["mdl"], e["frw"]["lft"], e["lft"]["frw"], e["lft"]["mdl"], e["lft"]["bkw"],e["bkw"]["lft"], e["bkw"]["mdl"], e["bkw"]["rgt"], e["rgt"]["bkw"], e["rgt"]["mdl"], e["rgt"]["frw"], e["frw"]["rgt"]];
        let ldr = l["distances"];
        let fldr = fl["distances_x2"];
//        console.log(pts);
//		console.log(ldr);
//		console.log(fldr);
//		console.log(az)
		pts.push(pts[0]);
		ldr.push(ldr[0]);
		fldr.push(0);
		fldr.unshift(0);
		degrees = [360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0]
		// degrees = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360];
		// front_degrees = [75, 45, 15, 345, 315, 285];
		front_degrees = [270, 285, 315, 345, 15, 45, 75, 90];
		if (pts.length == 0) {
			console.error("Can't get coordinates!");
		} else {
		    $(".robotenv").remove();
			drawVisibleStacks(fldr, front_degrees, coords, "green");
			drawVisibleStacks(pts, degrees, coords, "red");
			drawVisibleStacks(ldr, degrees, coords, "blue");

		}
	// }).fail(function(err) {
	// 	console.error(err);
	// });

	return pts;
}

function pol2cart(rho, phi, coords) {
  let theta = orientation_angle;
  x = coords[0] + rho * Math.cos((phi + theta + coords[2])*(Math.PI/180));
  y = coords[1] + rho * Math.sin((phi + theta + coords[2])*(Math.PI/180));
  return [+x, +y];
}
function drawVisibleStacks(pts, degrees, coords, color) {
	let r = $("#layer-svg g");
    let d = d_divider;
    let x = start_point_x;
    let y = start_point_y
	for (let i = pts.length - 2; i >= 0 ; i--) {
		let coords_0 = pol2cart(pts[i+1], degrees[i+1], coords);
		let coords_1 = pol2cart(pts[i], degrees[i], coords);
		$(r[5]).append('<svg xmlns="http://www.w3.org/2000/svg"><line class="robotenv"' +
		'x1="'+((coords_0[0]/d)+x)+'" y1="'+((coords_0[1]/d)+y)+'" x2="'+((coords_1[0]/d)+x)+'" y2="'+((coords_1[1]/d)+y)+'"'+
		' stroke="'+color+'" fill="transparent" stroke-width="1"></line>');
		// $(r[5]).append('<svg xmlns="http://www.w3.org/2000/svg"><line class="robotenv"' +
		// 'x1="300" y1="50" x2="'+((coords_1[0]/d)+x)+'" y2="'+((coords_1[1]/50)+y)+'"'+
		// ' stroke="white" fill="transparent" stroke-width="1"></line>');
	}
}
//


//Функция отображения PopUp
function PopUpShow(){
    $("#popup1").show();
}
//Функция скрытия PopUp
function PopUpHide(){
    $("#popup1").hide();
}

function getCurDate(){
  var now = new Date();
  now.setMinutes(now.getMinutes() + 1)
  var year = now.getFullYear();
  var month = now.getMonth() +1;
  var day = now.getDate();
  var hour = now.getHours();
  var minute = now.getMinutes();
  var localDatetime = year.toString() + "-" + (month < 10 ? "0" + month.toString() : month) + "-" + 
                      (day < 10 ? "0" + day.toString() : day) + "T" + 
                      (hour < 10 ? "0" + hour.toString() : hour) + ":" +
                      (minute < 10 ? "0" + minute.toString() : minute)
  var datetimeField = document.getElementById("date");
  // var newLocalDatetime = add1Minute(localDatetime);
  // datetimeField.value = newLocalDatetime;
  // datetimeField.min = newLocalDatetime;
  datetimeField.value = localDatetime;
  datetimeField.min = localDatetime; 
};

// function add1Minute(dateString) {
//   for (let i = 15; i > 0; i--) {
//     if (isNumeric(dateString[i])) {
//       if (dateString[i] == '9') {
//         dateString[i] = '0';
//       }
//       else {
//         dateString[i] = toString(parseInt(dateString[i]) + 1);
//         return dateString;
//       }
//     }
//   }
// };



//     var now = new Date();
//     var utcString = now.toISOString().substring(0,19);
//     var year = now.getFullYear();
//     var month = now.getMonth() + 1;
//     var day = now.getDate();
//     var hour = now.getHours();
//     var minute = now.getMinutes();
//     var second = now.getSeconds();
//     var localDatetime = year + "-" +
//                       (month < 10 ? "0" + month.toString() : month) + "-" +
//                       (day < 10 ? "0" + day.toString() : day) + "T" +
//                       (hour < 10 ? "0" + hour.toString() : hour) + ":" +
//                       (minute < 10 ? "0" + minute.toString() : minute) +
//                       utcString.substring(16,19);
//     var datetimeField = document.getElementById("date");
//     datetimeField.value = localDatetime;
// };

function crontab() {
  var task = $("#task").val();
  var repeat = $("#repeat").val();
  var datetime = new Date($("#date").val());
  // var datetime = new Date(document.getElementById("date").value);
  var now = new Date();
  console.log(datetime);
  if (datetime <= now) {
    alert("Это время уже прошло! Выберите другое")
  }
  else {
    let weekday = datetime.getDay();
    // console.log(weekday);
    let day = datetime.getDate();
    // console.log(day);
    let month = datetime.getMonth() + 1;
    // console.log(month);
    // let year = datetime.getYear()+1900;
    // console.log(year);
    let hours = datetime.getHours();
    // console.log(hours);
    let minutes = datetime.getMinutes();
    // console.log(minutes);

    // Crontab func arguments are: m,h,d,mon,dow,task
    let str = ["/schedule/add", minutes, hours, day, month, weekday, task, repeat].join("/") 
    console.log(str);
    $.post(str, function(resp) {
        console.log(resp);
        if (resp.length == 0) {
            console.error("Can't get crontab!");
        }
    }).done(function() {
        drawSchedule()
    }).fail(function(err) {
        console.error(err);
    });
  }
}

// // ************ WITH PROMISE ************
// function getSchedule() {
//   return new Promise((resolve, reject) => {
//     $.post("/schedule/get", function(resp) {
//       console.log("post resp = " + resp);
//       if (resp.length == 0) {
//         console.error("Can't get schedule!");
//       };
//     }).done(function(resp) {
//       console.log("done: resp = " + resp)
//       resolve(resp);
//     }).fail(function(error) {
//       console.log("fail: fail in getSchedule")
//       reject(error);
//     });
//   });
// };


// ************ WITHOUT PROMISE ************
function drawSchedule() {
  $.post("/schedule/get", function(resp) {
      console.log("ajax resp = " + resp);
      if (resp.length == 0) {
        console.error("Can't get schedule!");
      }
  })
  .done(function(resp) {
    console.log("this goes to drawTable: " + resp)
    drawTable(resp)
  })
  .fail(function(err) {
    console.error(err);
  });
};


function drawTable(resp) {
  let table = document.querySelector('#schedule-table');
  table.innerHTML = '';
  // let resp = resp
  console.log("resp in drawTable = " + resp);
  console.log("resp.length = " + resp.length)
  if (resp.length == 0) {
    let text = document.createElement('text');
    text.innerText = 'Пока нет запланированных задач';
    table.appendChild(text);
  } else {
    for (let i = 0; i < resp.length; i++) {
      let tr = document.createElement('tr');
      let rdata = resp[i];
      let tdtype = document.createElement('td');
      tr.appendChild(tdtype);
      tdtype.innerText = getWashTypeFromResp(rdata);
      let tddate = document.createElement('td');
      tr.appendChild(tddate);
      tddate.innerText = getDateFromResp(rdata);
      let tdtime = document.createElement('td');
      tr.appendChild(tdtime);
      // tdtime.innerText = rdata.hour + ":" + rdata.minute;
      tdtime.innerText = (rdata.hour.length < 2 ? "0" + rdata.hour : rdata.hour) + ":" + (rdata.minute.length < 2 ? "0" + rdata.minute : rdata.minute);
      let tdbutton = document.createElement('td');
      tr.appendChild(tdbutton);
      let button = document.createElement('button');
      console.log(rdata)
      tdbutton.appendChild(button);
      button.innerText = '⮾';
      button.id = 'deleteJob';
      button.setAttribute('onclick', 'deleteJob(' + JSON.stringify(rdata) + ')');
      button.class = 'is-block is-danger is-large is-fullwidth'; 
      table.appendChild(tr);
      }  
  }
};


// // ************ WITH PROMISE ************
// function drawSchedule(resp) {
//   let table = document.querySelector('#table');
//   table.innerHTML = '';
//   // let resp = [{"day":"15","day_of_week":"6","hour":"20","index":"12","minute":"00","month":"7","wash_type":"dry"},{"day":"14","day_of_week":"5","hour":"21","index":"11","minute":"30","month":"7","wash_type":"dry"},{"day":"*","day_of_week":"5","hour":"17","minute":"35","month":"*","wash_type":"wet"},{"day":"1","day_of_week":"*","hour":"0","minute":"0","month":"*","wash_type":"wet"},{"day":"1","day_of_week":"*","hour":"20","minute":"0","month":"*","wash_type":"wet"}];
//   getSchedule().then((resp) => {
//     console.log(resp)
//     if (resp.length == 0) {
//       let text = document.createElement('text');
//       text.innerText = 'Пока нет запланированных задач';
//       table.appendChild(text);
//     } 
//     else {
//       for (let i = 0; i < resp.length; i++) {
//         let tr = document.createElement('tr');
//         let rdata = resp[i];
//         let tdtype = document.createElement('td');
//         tr.appendChild(tdtype);
//         tdtype.innerText = getWashTypeFromResp(rdata);
//         let tddate = document.createElement('td');
//         tr.appendChild(tddate);
//         tddate.innerText = getDateFromResp(rdata);
//         let tdtime = document.createElement('td');
//         tr.appendChild(tdtime);
//         tdtime.innerText = rdata.hour + ":" + rdata.minute;
//         let tdbutton = document.createElement('td');
//         tr.appendChild(tdbutton);
//         let button = document.createElement('button');
//         console.log(rdata)
//         tdbutton.appendChild(button);
//         button.innerText = '⮾';
//         button.id = 'deleteJob';
//         button.setAttribute('onclick', 'deleteJob(' + JSON.stringify(rdata) + ')');
//         button.class = 'is-block is-success is-large is-fullwidth'; 
//         table.appendChild(tr);
//         };  
//     }
//   });
// };

function deleteJob(rdata){
  let confirmString = "Удалить задачу " + getWashTypeFromResp(rdata) + " " + getDateFromResp(rdata) + " " + 
                  (rdata["hour"].length < 2 ? "0" + rdata["hour"] : rdata["hour"]) + ":" + 
                  (rdata["minute"].length < 2 ? "0" + rdata["minute"] : rdata["minute"]) + "?"
  let isConfirmed = confirm(confirmString);
  if (isConfirmed == true) {
    console.log('preparing to delete '+ JSON.stringify(rdata ));
    console.log('rdata = '+ rdata);
    console.log(rdata);
    console.log(rdata["index"]);
    if (rdata.index) {
      sendString = '/schedule/delete/at/' + rdata.index
    }
    else {
      // /schedule/delete/crontab/<minute>/<hour>/<day>/<month>/<day_of_week>
      sendString = ['/schedule/delete/crontab', rdata["minute"], rdata["hour"],rdata["day"],rdata["month"],rdata["day_of_week"]].join("/") 
    }
    console.log(sendString);
    $.post(sendString, function(resp) {
      console.log("ajax resp = " + resp);
      if (resp.length == 0) {
        console.error("Can't get schedule!");
      }
    })
    .done(function(resp) {
      console.log("this goes to drawTable: " + resp)
      drawSchedule()
    })
    .fail(function(err) {
      console.error(err);
    });
  } else {
    console.log("Deletion canceled")
  }
};

function getWashTypeFromResp(elem) {
  let res = ""
  for (let i = 0; i < $washTypes.length; i++) {
    if ($washTypes[i]['id'] == elem['wash_type']) {
      res = $washTypes[i]['name']
    }
  }
  if (res.length < 1) {
    console.error("[ ERROR ] No wash type " + elem['wash_type'] + " found!");
  }
  return res;
};

function getDateFromResp(data){
  let dateStr = '';
  if (data.month == '*') {
    if (data.day == '*') {
      if (data.day_of_week == '*') {
        dateStr = 'Каждый день';
      }
      else if (data.day_of_week == '0') {
        dateStr = 'Каждое воскресенье';
      }
      else if (data.day_of_week == '1') {
        dateStr = 'Каждый понедельник';
      }
      else if (data.day_of_week == '2') {
        dateStr = 'Каждый вторник';
      }
      else if (data.day_of_week == '3') {
        dateStr = 'Каждую среду';
      }
      else if (data.day_of_week == '4') {
        dateStr = 'Каждый четверг';
      }
      else if (data.day_of_week == '5') {
        dateStr = 'Каждую пятницу';
      }
      else if (data.day_of_week == '6') {
        dateStr = 'Каждую субботу';
      }
    }
    else {
      dateStr = 'Каждый месяц ' + data.day + ' числа';
    }
  }
  else {
    let dayStr = '';
    if (data.day_of_week == '0') {
      dayStr = 'Воскресенье ';
    }
    else if (data.day_of_week == '1') {
      dayStr = 'Понедельник ';
    }
    else if (data.day_of_week == '2') {
      dayStr = 'Вторник ';
    }
    else if (data.day_of_week == '3') {
      dayStr = 'Среда ';
    }
    else if (data.day_of_week == '4') {
      dayStr = 'Четверг ';
    }
    else if (data.day_of_week == '5') {
      dayStr = 'Пятница ';
    }
    else if (data.day_of_week == '6') {
      dayStr = 'Суббота ';
    }
    dateStr = dayStr + (data["day"].length < 2 ? "0" + data["day"] : data["day"]) + "." + (data["month"].length < 2 ? "0" + data["month"] : data["month"]);
  }
  return dateStr;
};

// function getDateFromResp(data){
//   let dateStr = (data["day"].length < 2 ? "0" + data["day"] : data["day"]) + "." + (data["month"].length < 2 ? "0" + data["month"] : data["month"]);
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '*') {
//     dateStr = 'Каждый день';
//   };
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '0') {
//     dateStr = 'Каждое воскресенье';
//   };
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '1') {
//     dateStr = 'Каждый понедельник';
//   };
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '2') {
//     dateStr = 'Каждый ворник';
//   };
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '3') {
//     dateStr = 'Каждую среду';
//   };
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '4') {
//     dateStr = 'Каждый четверг';
//   };
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '5') {
//     dateStr = 'Каждую пятницу';
//   };
//   if (data.day == '*' && data.month == '*' && data.day_of_week == '6') {
//     dateStr = 'Каждую субботу';
//   };
//   if (data.day != '*' && data.month == '*' && data.day_of_week == '*') {
//     dateStr = 'Каждый месяц ' + data.day + ' числа';
//   };
//   return dateStr;
// };

function makeSelectForm() {
  let task = document.querySelector('#task');
  for (let i = 0; i < $washTypes.length; i++) {
    let ov = document.createElement('option');
    ov.value = $washTypes[i].id;
    ov.innerText = $washTypes[i].name;
    task.appendChild(ov);
  };
};
function getWashTypes() {
  // $washTypes = [{'id': 'dry', 'name': 'Бухая уборка', 'command': 'wash_start'},{'id': 'wet', 'name': 'Влажная фантазия', 'command': 'wash_start'}]
  $.post("/schedule/wash_types", function(resp) {
      console.log('getWashTypes().resp = ' + resp);
      if (resp.length == 0) {
          console.error("Can't get wash types!");
      }
  }).done(function(resp) {
      console.log('done resp' + resp);
      $washTypes = resp;
      makeSelectForm();

  }).fail(function(err) {
      console.error(err);
  });
};



$(document).ready(function() {
  getWashTypes();
  drawSchedule();
  setTimeout(() => {
    console.log('washTypes in document.ready: '+ $washTypes);
  }, 5000);
  console.log($washTypes.length);
  // setTimeout(() => {let task = document.querySelector('#task');
  // for (let i = 0; i < $washTypesLength; i++) {
  //   let ov = document.createElement('option');
  //   ov.value = $washTypes[i].id
  //   ov.innerText = $washTypes[i].name
  //   task.appendChild(ov);
  // }}, 5000);
  //Скрыть PopUp при загрузке страницы    
    PopUpHide();

   let t = setInterval(getEnvironment, 3000);
});
