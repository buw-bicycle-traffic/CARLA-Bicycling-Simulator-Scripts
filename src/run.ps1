# manual_control_simulator_concurrent.ps1

# Start each Python command as a background job
$job1 = Start-Job -ScriptBlock { py .\manual_control_simulator.py -x 422 }
Start-Sleep -Seconds 2

$job2 = Start-Job -ScriptBlock { py .\AdditionalDisplay.py --display_name left }
$job3 = Start-Job -ScriptBlock { py .\AdditionalDisplay.py --display_name right }
$job4 = Start-Job -ScriptBlock { py .\AdditionalDisplay.py --display_name center }
$job5 = Start-Job -ScriptBlock { & "F:\G\CppAnt\DEMO_DLL.exe" 0 1 75 }
# Optionally, wait for all jobs to complete
Wait-Job $job1, $job2, $job3, $job4

# Clean up the jobs
Remove-Job $job1, $job2, $job3, $job4
