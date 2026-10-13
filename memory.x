MEMORY
{
  /* nRF52833: 512K flash, 128K RAM. The SoftDevice Controller is linked into
     the application, so the full ranges are available. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
