;

function getState() {
	$.get("/system/state/get", function(resp) {
		console.log(resp);
		document.getElementsByClassName('system')[0].innerHTML = JSON.stringify(resp);

		// TODO: make a state checking and map it to button values
	}).fail(function(err) {
		console.error(err);
	});
}

// function updateDebugInfo() {
//     $.post("/state/get", function(resp) {
//         console.log(resp);
//         if (resp.length == 0) {
//             console.error("Can't get environment data!");
//         } else {
//             console.log(resp["environment"]);
//             console.log(resp["lidar"]);
//             console.log(resp["lidar"]["distances"]);
//             console.log(resp["front_lidar"])
//         }
//   }).fail(function(err) {
//       console.error(err);
//   });
// }

function showDebugInfo(info) {

}

// $(document).ready(function() {
// 	let t = setInterval(getState, 3000);
// });