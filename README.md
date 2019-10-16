# deepracer-tools
Some tools for AWS deepracer

## Auto Submission

AWS DeepRacer provides a virtual circuit where we can submit our model. The same model can get difference lap time for every submission, therefore people resubmit it several time to get the fastest possible lap time. There is currently no restriction how often we can submit our model, however we have to wait 30 minutes between submission. All of these submissions have been done using a web browser since there is currently no DeepRacer capability in AWS cli. Some people use Selenium web browser plugin to automate this task. 

The aim of this DeepRacer Auto Submission Tool is, as the title says, to resubmit automatically a DeepRacer model every 30 minutes using only command line interface. It consists of several shell scripts and requires only curl, a command line tool for data transfer, and jq, a lightweight command line JSON processor.

### How to use it
1. First Install the requirements (curl is normally already installed on your machine):
    ```
    $ sudo apt install curl jq
    ```
2. Then you have to clone this repository:
    ```
    $ git clone https://github.com/cahya-wirawan/deepracer-tools.git
    ```
3. Change to the tool directory:
    ```
    $ cd deepracer-tools/auto-submission
    ```
4. Copy the configuration template, and update it with your AWS credential:
    ```
    $ cp aws-deepracer.cf.sample aws-deepracer.cf
    $ cat aws-deepracer.cf
      ACCOUNTID="123456789012"
      USERNAME="my-username"
      PASSWORD=""
      EMAIL="my-username@example.com"
    ```
    If the password is empty, the script will ask you for it. The email address is used to inform you when the result of
    the submission is arrive or if there is any issue with the submission.
 5. Login to the AWS
    ```
    $ source ./aws-authenticate
    ``` 
    This authentication process will save the authentication tokens (aws-userInfo and aws-creds) and CSRF token in your 
    environment variables for further usage, so you don't need to  authenticate for every scripts you run next time 
    until they are expire. The tokens will be valid for around 6 hours, after expired, you need to reauthenticate again 
    using the script above. 
 6. Run the Auto Submission script and wait for email notification :-)
    ```
    $ ./aws-submitModel "Your deepracer model name"
    ``` 
 
