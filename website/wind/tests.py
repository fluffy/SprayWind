from django.test import TestCase


class SimpleTest(TestCase):
    def testOne(self):
        self.assertEqual(42,42)
