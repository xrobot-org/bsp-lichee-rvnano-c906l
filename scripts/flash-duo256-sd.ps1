param(
    [Parameter(Mandatory = $true)]
    [string]$ImagePath,

    [int]$DiskNumber = 2,

    [string]$LogPath = 'D:\Projects\sophgo-sg200x-debian\output\duo256-flash.log'
)

$ErrorActionPreference = 'Stop'
Start-Transcript -LiteralPath $LogPath -Force

try {
    $disk = Get-Disk -Number $DiskNumber
    $image = Get-Item -LiteralPath $ImagePath
    if ($disk.BusType -ne 'USB' -or $disk.Size -lt $image.Length -or $disk.IsBoot -or $disk.IsSystem) {
        throw "Refusing to write: Disk $DiskNumber does not match the verified USB SD card."
    }

    # Windows otherwise mounts the new FAT boot partition immediately after
    # its MBR is written, which blocks the rest of a raw removable-media write.
    & mountvol.exe /n
    Get-Partition -DiskNumber $DiskNumber | Where-Object DriveLetter | ForEach-Object {
        & mountvol.exe ("$($_.DriveLetter):") /p
    }

    $input = $null
    $output = $null
    try {
        $input = [System.IO.File]::Open($ImagePath, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::Read)
        $output = [System.IO.File]::Open("\\.\PhysicalDrive$DiskNumber", [System.IO.FileMode]::Open, [System.IO.FileAccess]::Write, [System.IO.FileShare]::ReadWrite)
        $buffer = New-Object byte[] (4MB)
        [Int64]$written = 0
        while (($read = $input.Read($buffer, 0, $buffer.Length)) -gt 0) {
            $output.Write($buffer, 0, $read)
            $written += $read
        }
        $output.Flush($true)
        if ($written -ne $image.Length) {
            throw "Incomplete write: $written of $($image.Length) bytes."
        }
        "WROTE_BYTES=$written"
    }
    finally {
        if ($output) { $output.Dispose() }
        if ($input) { $input.Dispose() }
    }

    $checkLength = [Math]::Min([Int64](64MB), $image.Length)
    $hashAlgorithm = [Security.Cryptography.SHA256]::Create()
    $sourceBytes = New-Object byte[] $checkLength
    $sourceStream = [System.IO.File]::OpenRead($ImagePath)
    [void]$sourceStream.Read($sourceBytes, 0, $sourceBytes.Length)
    $sourceStream.Dispose()
    $sourceHash = [BitConverter]::ToString($hashAlgorithm.ComputeHash($sourceBytes)).Replace('-', '')

    $diskBytes = New-Object byte[] $checkLength
    $diskStream = [System.IO.File]::Open("\\.\PhysicalDrive$DiskNumber", [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::ReadWrite)
    [void]$diskStream.Read($diskBytes, 0, $diskBytes.Length)
    $diskStream.Dispose()
    $diskHash = [BitConverter]::ToString($hashAlgorithm.ComputeHash($diskBytes)).Replace('-', '')
    $hashAlgorithm.Dispose()

    if ($sourceHash -ne $diskHash) {
        throw "Verification failed: source=$sourceHash disk=$diskHash"
    }
    "PREFIX_SHA256=$diskHash"
    'FLASH_SUCCESS'
}
catch {
    'FLASH_ERROR'
    $_ | Format-List * -Force
    exit 1
}
finally {
    & mountvol.exe /e
    Stop-Transcript
}
