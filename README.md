# DeepRacer Tools
Some tools for AWS DeepRacer

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
    until they are expire. The tokens will be valid for around 10 hours, after expired, you need to reauthenticate again 
    using the script above. 
 6. Run the Auto Submission script and wait for email notification :-) It assumes that you have already uploaded your 
    model trained locally or the model has been trained in the AWS cloud. 
    ```
    $ ./aws-submitModel "Your deepracer model name"
    ``` 
 
### List of the tools
The tool has not only the main script to auto resubmit the model, but include also some other scripts. Following is 
the complete list of the scripts:
1. aws-authenticate

   This is the first script to be executed before you can use other scripts. 
   Run it as follow: 
   ```
   $ source ./aws-authenticate
   ``` 
2. aws-createLeaderboardSubmission

   This script submit a model. An example to run it and its output if you have submitted a model in the last 30 minutes:
   ```
   $ ./aws-createLeaderboardSubmission DR-Local
   {
     "__type": "TooManyRequestsException",
     "Message": "You have exceeded the number leaderboard submission requests allowed on your account."
   }
   ```
3. aws-getLatestUserSubmission
   ```
   $ ./aws-getLatestUserSubmission
   {
     "LeaderboardSubmission": {
       "Alias": "DodolGarut",
       "AvgLapTime": 0,
       "JobName": "sim-9jk8hj5zmjlm",
       "LapCount": 0,
       "LeaderboardSubmissionStatusType": "RUNNING",
       "ModelArn": "arn:aws:deepracer:us-east-1:123456789012:model/reinforcement_learning/DR-Local",
       "SubmissionTime": 1571232665626
     }
   }
   ```
4. aws-getRankedUserSubmission
   ```
   $ ./aws-getRankedUserSubmission
   {
     "LeaderboardSubmission": {
       "Alias": "DodolGarut",
       "AvgLapTime": 10254,
       "LeaderboardSubmissionStatusType": "SUCCESS",
       "ModelArn": "arn:aws:deepracer:us-east-1:123456789012:model/reinforcement_learning/DR-Local",
       "Rank": 50,
       "SubmissionTime": 1571226412408
     }
   }
   ```
5. aws-listLeaderboardSubmissions
   ```
   $ ./aws-listLeaderboardSubmissions
   {
     "LeaderboardSubmissions": [
       {
         "Alias": "Karl-NAB",
         "AvgLapTime": 7555,
         "LeaderboardSubmissionStatusType": "SUCCESS",
         "Rank": 1,
         "SubmissionTime": 1571051009345
       },
       {
         "Alias": "JJ",
         "AvgLapTime": 7745,
         "LeaderboardSubmissionStatusType": "SUCCESS",
         "Rank": 2,
         "SubmissionTime": 1571195290542
       },
       {
         "Alias": "D4D-test",
         "AvgLapTime": 8305,
         "LeaderboardSubmissionStatusType": "SUCCESS",
         "Rank": 3,
         "SubmissionTime": 1571128249473
       },
       ....
       {
         "Alias": "Shendy",
         "AvgLapTime": 11476,
         "LeaderboardSubmissionStatusType": "SUCCESS",
         "Rank": 99,
         "SubmissionTime": 1570583780470
       },
       {
         "Alias": "PolishThunder",
         "AvgLapTime": 11506,
         "LeaderboardSubmissionStatusType": "SUCCESS",
         "Rank": 100,
         "SubmissionTime": 1571107235595
       }
     ],
     "NextToken": "XYZ..."
   } 
   ```
6. aws-listLeaderboards
   ```
   $ ./aws-listLeaderboards
   {
     "Leaderboards": [
       {
         "Arn": "arn:aws:deepracer:us-east-1::leaderboard/season-2019-08",
         "CloseTime": 1567295999000,
         "Description": "The August race is now open and is the fourth stop on the Virtual Circuit World Tour. Create and train a model to master the Shanghai Sudu track and submit your model for a chance to win an all-expenses paid trip to Las Vegas, NV, to compete for the Championship Cup at re:Invent 2019!",
         "ImageUrl": "https://deepracer-managed-resources-us-east-1.s3.amazonaws.com/leaderboard-resources/china_leaderboard.svg",
         "LaunchTime": 1564642800000,
         "MinimumLaps": 1,
         "Name": "Shanghai Sudu",
         "ParticipantCount": 1375,
         "Status": "CLOSED",
         "TrackArn": "arn:aws:deepracer:us-east-1::track/ChinaAlt_track",
         "TrackImageUrl": "https://deepracer-managed-resources-us-east-1.s3.amazonaws.com/track-resources/chinaalt_track.svg"
       },
       {
         "Arn": "arn:aws:deepracer:us-east-1::leaderboard/season-2019-06",
         "CloseTime": 1561964399000,
         "Description": "Continue racing and building up League points in the Kumo Torakku, the second stop in the Virtual Circuit World Tour. The race winner and season top point getters will win an expenses paid trip to compete for the Championship Cup at re:Invent 2019.",
         "ImageUrl": "https://deepracer-managed-resources-us-east-1.s3.amazonaws.com/leaderboard-resources/japan_leaderboard.svg",
         "LaunchTime": 1559595600000,
         "MinimumLaps": 1,
         "Name": "Kumo Torakku",
         "ParticipantCount": 572,
         "Status": "CLOSED",
         "TrackArn": "arn:aws:deepracer:us-east-1::track/Tokyo_Racing_track",
         "TrackImageUrl": "https://deepracer-managed-resources-us-east-1.s3.amazonaws.com/track-resources/tokyo_racing_track.svg"
       },
   ...
   } 
   ```

7. aws-submitModel
   ```
   $ ./aws-submitModel DR-Local
   ...
   Mi Okt 16 13:15:05 CEST 2019
   "RUNNING" 0
   "RUNNING" 0
   "RUNNING" 0
   "SUCCESS" 10802
   Mi Okt 16 14:48:15 CEST 2019
   "RUNNING" 0
   "RUNNING" 0
   "RUNNING" 0
   "RUNNING" 0
   "SUCCESS" 10254
   ...
 
   ```
8. aws-getModel

   The script download a model from AWS as compressed tar file. It requires additional AWS cli.
   ```
   ./aws-getModel DR-Local
   Completed 16.0 MiB/20.5 MiB (3.6 MiB/s) with 1 file(s) remaining                                      
   ```
   
9. aws-execCurl

   The script is only used inside other scripts, should not run directly.
   
### TODO List
* Gradually increase the speed in action space and resubmit until the model doesn't get any laps time improvement. 
This approach could get the best out of the model without retraining it.
* Re-authenticate automatically after the tokens expire
* Upload automatically the latest model from local training before submission.