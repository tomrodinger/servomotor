"""Release metadata must select flashable images, never checksum/notes sidecars."""
from pathlib import Path
import tempfile
import unittest

from M17_generate_api_documentation import APIDocumentationGenerator


class FirmwareSelectionTest(unittest.TestCase):
    def test_sidecars_and_directories_cannot_be_selected_as_firmware(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = 'servomotor_M17_fw0.16.0.0_scc3_hw1.5.firmware'
            # Create sidecars first to expose filesystem-order-dependent ties.
            for name in [release + '.sha256',
                         'servomotor_M17_fw99.0.0.0_scc3_hw1.5.release_notes.txt',
                         'servomotor_M17_fw0.15.12.1_scc3_hw1.5.firmware', release]:
                (root / name).write_text('fixture')
            (root / 'servomotor_M17_fw98.0.0.0_scc3_hw1.5.firmware').mkdir()
            products = root / 'VALID_PRODUCTS.txt'
            products.write_text('servomotor_M17\n')
            generator = APIDocumentationGenerator()
            generator.firmware_dir_path = directory
            generator.valid_products_path = str(products)
            self.assertEqual(generator.find_latest_firmware(), {'Model M17': release})


if __name__ == '__main__':
    unittest.main()
