<!DOCTYPE html>
<html>
	<head>
		<title>md49data</title>
	</head>

	<body>
		<?php
		$db = new PDO('sqlite:md49data.db');
		$sql = 'SELECT * From md49data';
		$result = $db->query($sql);
		echo displayResult($result);

		function displayResult($result) {
			$r = '';

			$r .= '<table border="1" style="width:50%">';
			$r .= '<tr>';
				$r .=  '<th>SpeedL</th>';
				$r .=  '<th>SpeedR</th>';
			$r .= '</tr>';
			while($row = $result->fetchObject()) {
				$r .= '<tr>';
				$r .= '<td style="background-color:#eee; padding:3px">' . htmlspecialchars($row->SpeedL) . '</td>';
				$r .= '<td style="background-color:#eee; padding:3px">' . htmlspecialchars($row->SpeedR) . '</td>';
				$r .= '</tr>';
			}
			$r .= '</table>';

			return $r;
		}
		?>	
		
	</body>
</html>

