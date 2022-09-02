from setuptools import setup, find_packages

requires = ["twine>=3.8.0",
            "wheel>=0.37.1"]

setup(
    name='emuemuJP',
    version='0.0.3',
    packages=find_packages(),

    author='emuemuJP',
    author_email='k.matsumoto.0807@gmail.com',

    description='This is a package for me.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',

    python_requires='~=3.6',
    install_requires=requires,
    classifiers=[
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Operating System :: OS Independent',
    ],


)