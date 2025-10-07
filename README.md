# Presage Technologies Interview

## Heart Rate from Video

### Task
Extract the heart rate from the person in `codingtest.mov` using **Kotlin**, **C++**, or **Swift** and any packages you like.

### Recommended Approach
1. **Average the pixels** over a portion of the forehead for the green channel vs. time (the heart rate is strongest in the green channel).
2. **Perform a Fourier analysis** on the green channel signal vs. time to extract the heart rate:
	- Look for the peak in the Power Spectral Density (PSD) within a reasonable heart-rate range.
	- Ignore other prominent peaks outside of physiological ranges.

### Deliverables
- A small project or script that prints the estimated **BPM**.
- A short README with:
	- How to run it
	- The BPM you measured on the clip

### Submission
Reply back to the email with either a **GitHub link** or a **zip** of the code + README.

### Time Estimate
This should take about **1 hour**.